package cube_sorter

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
)

// errStopped is returned by an in-progress operation when stop() is called.
var errStopped = errors.New("operation stopped")

type armState string

const (
	stateIdle      armState = "idle"
	stateSearching armState = "searching_for_objects"
	stateDetected  armState = "objects_detected"
	statePicking   armState = "picking"
	statePlacing   armState = "placing"
	stateStopped   armState = "stopped"
	stateResetting armState = "resetting"
)

type cmdKind int

const (
	cmdStart cmdKind = iota
	cmdResume
)

var pickConstraints = motionplan.Constraints{
	LinearConstraint: []motionplan.LinearConstraint{
		{
			LineToleranceMm:          5,
			OrientationToleranceDegs: 1,
		},
	},
}

// Motion planning sees a live snapshot of the frame system, so an IK failure
// often just means the other arm happens to be parked in the way. Back off and
// try again — by the next attempt the other worker has usually moved on.
const (
	moveRetryAttempts = 5
	moveRetryBackoff  = 750 * time.Millisecond
)

// If a whole pick attempt fails after the inner retries, restart the cycle
// from detection rather than discarding the block. Bounded so a persistent
// obstacle still surfaces eventually.
const cycleRestartLimit = 2

// armWorker owns one arm and runs its pick-and-place cycle on a dedicated
// goroutine. Arm motion is planned on the gripper frame (so the frame system
// handles the gripper's mounting offset and finger geometry) and serialized via
// the shared motionMu so only one arm moves at a time. The worker mutex only
// guards in-memory state, so status and stop stay responsive.
type armWorker struct {
	name   string
	logger logging.Logger

	// deps
	arm       arm.Arm
	cam       camera.Camera
	gripper   gripper.Gripper
	segmenter vision.Service
	startPose toggleswitch.Switch
	zones     map[string]*zoneState

	// geometry
	cubeHeight   float64
	graspZOffset float64
	approachYaw  float64

	// shared
	motion   motion.Service
	client   robot.Robot
	motionMu *sync.Mutex // serializes arm motion: one arm moves at a time

	// lifecycle
	parentCtx context.Context
	cmdCh     chan cmdKind

	// interrupt control
	stopped  atomic.Bool
	opMu     sync.Mutex
	opCancel context.CancelFunc

	// in-memory state (short holds only)
	mu       sync.RWMutex
	state    armState
	detected []DetectedObject
}

func (w *armWorker) gripperName() string {
	return w.gripper.Name().ShortName()
}

// run is the worker's event loop; one cycle per start/resume command.
func (w *armWorker) run(wg *sync.WaitGroup) {
	defer wg.Done()
	for {
		select {
		case <-w.parentCtx.Done():
			return
		case kind := <-w.cmdCh:
			w.runCycle(kind)
		}
	}
}

// trigger requests a new cycle without blocking; duplicate requests while a
// cycle is already running are dropped.
func (w *armWorker) trigger(kind cmdKind) {
	select {
	case w.cmdCh <- kind:
	default:
	}
}

func (w *armWorker) runCycle(kind cmdKind) {
	ctx := w.newOpCtx()
	w.stopped.Store(false)

	dropHeld := kind == cmdResume
	for restart := 0; ; restart++ {
		outcome := w.runCycleAttempt(ctx, dropHeld)
		if outcome == cycleDone {
			return
		}
		if outcome == cycleComplete {
			break
		}
		// cyclePickFailed: a pick errored after the inner retries gave up.
		// Re-detect and try again from a clean slate — by then the other arm
		// has typically moved out of the way.
		if restart >= cycleRestartLimit {
			w.logger.Warnf("[%s] pick failed %d times; giving up on this cycle", w.name, restart+1)
			break
		}
		w.logger.Infof("[%s] restarting cycle from detection (restart %d/%d)", w.name, restart+1, cycleRestartLimit)
		// A partial pick may have left a block in the gripper; drop it.
		dropHeld = true
	}

	w.setState(stateIdle)
}

type cycleOutcome int

const (
	cycleComplete   cycleOutcome = iota // all detected objects processed (success or per-label drop)
	cycleDone                           // terminated (interrupt or unrecoverable error); caller should return
	cyclePickFailed                     // pickOne failed; caller may restart from detection
)

func (w *armWorker) runCycleAttempt(ctx context.Context, dropHeld bool) cycleOutcome {
	if err := w.detectFromStart(ctx, dropHeld); err != nil {
		w.handleCycleErr("detection", err)
		return cycleDone
	}

	// Nothing to pick — skip prepareZones (which would drive to inspect pose for
	// senseZone) and leave the arm parked at start. placeInZone preps lazily on
	// the first cycle that has detections.
	if _, ok := w.nextLabel(); !ok {
		w.logger.Infof("[%s] no owned objects detected; skipping zone preparation, staying at start", w.name)
		return cycleComplete
	}

	if err := w.prepareZones(ctx); err != nil {
		w.handleCycleErr("zone preparation", err)
		return cycleDone
	}

	for {
		if err := w.checkInterrupt(ctx); err != nil {
			w.logger.Infof("[%s] cycle interrupted before next pick", w.name)
			return cycleDone
		}
		label, ok := w.nextLabel()
		if !ok {
			return cycleComplete
		}
		if err := w.pickOne(ctx, label); err != nil {
			if w.interrupted(ctx, err) {
				w.logger.Infof("[%s] pick interrupted", w.name)
				return cycleDone
			}
			w.logger.Errorf("[%s] failed to pick %q: %v", w.name, label, err)
			return cyclePickFailed
		}
		w.removeDetected(label)
	}
}

func (w *armWorker) handleCycleErr(phase string, err error) {
	if errors.Is(err, errStopped) || errors.Is(err, context.Canceled) {
		w.logger.Infof("[%s] %s interrupted", w.name, phase)
		return
	}
	w.logger.Warnf("[%s] %s failed: %v", w.name, phase, err)
	w.setState(stateIdle)
}

// detectFromStart drives the arm to its start pose, optionally drops a held
// block (on resume), then detects this arm's owned colors from its camera.
func (w *armWorker) detectFromStart(ctx context.Context, dropHeld bool) error {
	w.setState(stateSearching)

	if err := w.setSwitch(ctx, w.startPose); err != nil {
		return err
	}
	if err := w.sleep(ctx, 500*time.Millisecond); err != nil {
		return err
	}

	if dropHeld {
		w.logger.Infof("[%s] opening gripper to drop any held block", w.name)
		if err := w.gripper.Open(ctx, nil); err != nil {
			return err
		}
	}

	w.logger.Infof("[%s] detecting objects", w.name)
	objs, err := w.segmenter.GetObjectPointClouds(ctx, "", nil)
	if err != nil {
		return err
	}
	dets, err := w.segmenter.DetectionsFromCamera(ctx, "", nil)
	if err != nil {
		return err
	}

	detected := []DetectedObject{}
	for _, obj := range objs {
		label := obj.Geometry.Label()
		// Only track colors this arm owns (those with a zone).
		if _, owned := w.zones[label]; !owned {
			continue
		}
		for _, det := range dets {
			if det.Label() != label {
				continue
			}
			// Transform the object's point cloud into world and stash the
			// center so it's visible via get_detected_objects without
			// needing to attempt a pick. Failures here are non-fatal
			// (we still report the 2D detection); the pick path will
			// re-transform when it runs.
			// Transform into world NOW, while the camera is still at the
			// detection pose. We stash the world-frame object so the pick
			// path can use it directly — re-transforming later would use the
			// (by then) moved camera pose and yield a wrong world position.
			selected := *obj
			inWorld, terr := w.client.TransformPointCloud(ctx, &selected, w.cam.Name().ShortName(), "world")
			if terr != nil {
				w.logger.Warnf("[%s] failed to transform %q point cloud to world: %v", w.name, label, terr)
				break
			}
			md := inWorld.MetaData()
			worldCenter := md.Center()
			w.logger.Infof("[%s] detected %q at world %v", w.name, label, worldCenter)
			detected = append(detected, DetectedObject{Label: label, Object: selected, Detection: det, WorldPC: inWorld, WorldCenter: worldCenter})
			break
		}
	}

	w.mu.Lock()
	w.detected = detected
	w.state = stateDetected
	w.mu.Unlock()

	w.logger.Infof("[%s] detected %d owned object(s)", w.name, len(detected))
	return nil
}

func (w *armWorker) pickOne(ctx context.Context, label string) error {
	obj, ok := w.lookupDetected(label)
	if !ok {
		return fmt.Errorf("no detected object with label %q", label)
	}

	w.setState(statePicking)

	// Use the world-frame point cloud captured at detection time — the camera
	// has since moved (zone inspect poses), so re-transforming the camera-frame
	// cloud now would project it through the wrong camera pose.
	objInWorld := obj.WorldPC
	objMd := objInWorld.MetaData()
	pickPoint := objMd.Center()
	// The depth camera mostly sees the top face, so descend cube_height/2 below
	// the visible top to grab mid-block.
	pickPoint.Z = objMd.MaxZ - w.cubeHeight/2 + w.graspZOffset

	yaw := graspYawDegrees(objInWorld, w.approachYaw)
	orient := spatialmath.OrientationVectorDegrees{OZ: -1, Theta: yaw}
	w.logger.Infof("[%s] pick %q at %v yaw=%.1f", w.name, label, pickPoint, yaw)

	gripperName := w.gripperName()
	approachPoint := pickPoint
	approachPoint.Z += approachHeightMm
	approachPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(approachPoint, &orient))
	pickPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(pickPoint, &orient))

	// Approach from above (waypoint), then descend straight down.
	plan := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{
			gripperName: approachPose,
		},
		referenceframe.FrameSystemInputs{},
	)

	if err := w.checkInterrupt(ctx); err != nil {
		return err
	}
	if err := w.gripper.Open(ctx, nil); err != nil {
		return err
	}
	if err := w.moveGripper(ctx, pickPose, &pickConstraints, map[string]any{
		"waypoints": []any{plan.Serialize()},
	}); err != nil {
		return err
	}

	if err := w.sleep(ctx, 500*time.Millisecond); err != nil {
		return err
	}
	if grabbed, err := w.gripper.Grab(ctx, nil); err != nil {
		return err
	} else if !grabbed {
		w.logger.Warnf("[%s] gripper reported no object grabbed for %q", w.name, label)
	}
	if err := w.sleep(ctx, 250*time.Millisecond); err != nil {
		return err
	}

	// Lift.
	liftPoint := pickPoint
	liftPoint.Z += liftHeightMm
	liftPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(liftPoint, &orient))
	if err := w.moveGripper(ctx, liftPose, nil, nil); err != nil {
		return err
	}

	// Place into the next free cell of the color's zone.
	w.setState(statePlacing)
	if err := w.placeInZone(ctx, label); err != nil {
		return err
	}

	// Return home.
	return w.setSwitch(ctx, w.startPose)
}

// moveGripper plans motion on the gripper frame, under motionMu so only one arm
// moves at a time. Planning-collision errors are retried with backoff: the
// planner sees the live frame system, so an IK failure is usually just the
// other arm parked in the way and clears once it moves on.
func (w *armWorker) moveGripper(ctx context.Context, dest *referenceframe.PoseInFrame, constraints *motionplan.Constraints, extra map[string]any) error {
	var err error
	for attempt := 1; attempt <= moveRetryAttempts; attempt++ {
		err = w.tryMoveGripper(ctx, dest, constraints, extra)
		if err == nil {
			return nil
		}
		if !isPlanCollisionErr(err) || attempt == moveRetryAttempts {
			return err
		}
		w.logger.Infof("[%s] move blocked (attempt %d/%d), retrying in %v: %v",
			w.name, attempt, moveRetryAttempts, moveRetryBackoff, err)
		if sleepErr := w.sleep(ctx, moveRetryBackoff); sleepErr != nil {
			return sleepErr
		}
	}
	return err
}

func (w *armWorker) tryMoveGripper(ctx context.Context, dest *referenceframe.PoseInFrame, constraints *motionplan.Constraints, extra map[string]any) error {
	if err := w.checkInterrupt(ctx); err != nil {
		return err
	}
	w.motionMu.Lock()
	defer w.motionMu.Unlock()
	if err := w.checkInterrupt(ctx); err != nil {
		return err
	}
	w.logger.Infof("[%s] commanding arm motion: motion.Move to %v", w.name, dest.Pose().Point())
	_, err := w.motion.Move(ctx, motion.MoveReq{
		ComponentName: w.gripperName(),
		Destination:   dest,
		Constraints:   constraints,
		Extra:         extra,
	})
	return err
}

// isPlanCollisionErr matches motion-plan failures caused by frame-system state
// (a constraint violation or no valid IK solution) — the kind that typically
// clears once the other arm finishes its current motion.
func isPlanCollisionErr(err error) bool {
	if err == nil {
		return false
	}
	msg := err.Error()
	return strings.Contains(msg, "IK solutions failed constraints") ||
		strings.Contains(msg, "violation between")
}

// setSwitch drives the arm to a saved switch pose, under motionMu.
func (w *armWorker) setSwitch(ctx context.Context, sw toggleswitch.Switch) error {
	if err := w.checkInterrupt(ctx); err != nil {
		return err
	}
	w.motionMu.Lock()
	defer w.motionMu.Unlock()
	if err := w.checkInterrupt(ctx); err != nil {
		return err
	}
	w.logger.Infof("[%s] commanding arm motion: switch %q SetPosition", w.name, sw.Name().ShortName())
	return sw.SetPosition(ctx, 2, nil)
}

// detectOnly runs a single detection pass and returns the serialized objects.
func (w *armWorker) detectOnly() ([]any, error) {
	ctx := w.newOpCtx()
	w.stopped.Store(false)
	if err := w.detectFromStart(ctx, false); err != nil {
		return w.serializeDetected(), err
	}
	return w.serializeDetected(), nil
}

// pickByLabel synchronously picks a single object, detecting and preparing the
// zone first if needed.
func (w *armWorker) pickByLabel(label string) error {
	ctx := w.newOpCtx()
	w.stopped.Store(false)

	if _, ok := w.lookupDetected(label); !ok {
		if err := w.detectFromStart(ctx, false); err != nil {
			return err
		}
	}
	if _, ok := w.lookupDetected(label); !ok {
		return fmt.Errorf("object %q not detected", label)
	}
	if err := w.prepareZone(ctx, label); err != nil {
		return err
	}
	if err := w.pickOne(ctx, label); err != nil {
		return err
	}
	w.removeDetected(label)
	w.setState(stateIdle)
	return nil
}

// stop cancels in-flight motion, halts the arm, and flips the silence flag so
// no further commands are issued until resume.
func (w *armWorker) stop() error {
	w.stopped.Store(true)
	w.cancelOp()
	w.setState(stateStopped)

	ctx, cancel := context.WithTimeout(w.parentCtx, 5*time.Second)
	defer cancel()
	w.logger.Infof("[%s] stopping arm", w.name)
	return w.arm.Stop(ctx, nil)
}

// resume clears the silence flag and re-runs detect-and-continue.
func (w *armWorker) resume() {
	w.stopped.Store(false)
	w.trigger(cmdResume)
}

// reset stops the arm, opens the gripper, returns to start, and clears state.
func (w *armWorker) reset() error {
	w.stopped.Store(true)
	w.cancelOp()
	w.setState(stateResetting)

	ctx, cancel := context.WithTimeout(w.parentCtx, 30*time.Second)
	defer cancel()

	w.logger.Infof("[%s] reset: stopping arm", w.name)
	if err := w.arm.Stop(ctx, nil); err != nil {
		return err
	}
	if err := w.gripper.Open(ctx, nil); err != nil {
		return err
	}

	w.motionMu.Lock()
	w.logger.Infof("[%s] commanding arm motion: switch %q SetPosition (reset)", w.name, w.startPose.Name().ShortName())
	err := w.startPose.SetPosition(ctx, 2, nil)
	w.motionMu.Unlock()
	if err != nil {
		return err
	}

	w.mu.Lock()
	w.state = stateIdle
	w.detected = nil
	w.mu.Unlock()

	w.stopped.Store(false)
	return nil
}

// --- interrupt context helpers ---

func (w *armWorker) newOpCtx() context.Context {
	w.opMu.Lock()
	defer w.opMu.Unlock()
	ctx, cancel := context.WithCancel(w.parentCtx)
	w.opCancel = cancel
	return ctx
}

func (w *armWorker) cancelOp() {
	w.opMu.Lock()
	defer w.opMu.Unlock()
	if w.opCancel != nil {
		w.opCancel()
	}
}

func (w *armWorker) checkInterrupt(ctx context.Context) error {
	if w.stopped.Load() {
		return errStopped
	}
	return ctx.Err()
}

func (w *armWorker) interrupted(ctx context.Context, err error) bool {
	return errors.Is(err, errStopped) || errors.Is(err, context.Canceled) || ctx.Err() != nil
}

// sleep waits while honoring cancellation/stop.
func (w *armWorker) sleep(ctx context.Context, d time.Duration) error {
	if err := w.checkInterrupt(ctx); err != nil {
		return err
	}
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		if w.stopped.Load() {
			return errStopped
		}
		return nil
	}
}

// --- state accessors (short locks) ---

func (w *armWorker) setState(s armState) {
	w.mu.Lock()
	w.state = s
	w.mu.Unlock()
}

func (w *armWorker) status() armState {
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.state
}

func (w *armWorker) ownsLabel(label string) bool {
	_, ok := w.zones[label]
	return ok
}

func (w *armWorker) nextLabel() (string, bool) {
	w.mu.RLock()
	defer w.mu.RUnlock()
	if len(w.detected) == 0 {
		return "", false
	}
	return w.detected[0].Label, true
}

func (w *armWorker) lookupDetected(label string) (DetectedObject, bool) {
	w.mu.RLock()
	defer w.mu.RUnlock()
	for _, o := range w.detected {
		if o.Label == label {
			return o, true
		}
	}
	return DetectedObject{}, false
}

func (w *armWorker) removeDetected(label string) {
	w.mu.Lock()
	defer w.mu.Unlock()
	for i, o := range w.detected {
		if o.Label == label {
			w.detected = slices.Delete(w.detected, i, i+1)
			return
		}
	}
}

func (w *armWorker) serializeDetected() []any {
	w.mu.RLock()
	defer w.mu.RUnlock()
	out := []any{}
	for _, o := range w.detected {
		out = append(out, o.Serialize())
	}
	return out
}
