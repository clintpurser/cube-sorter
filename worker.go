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

// errGripperEmpty is returned by liftDetected when Grab reports no object was
// captured. The cycle loop treats this as a pick failure and restarts from
// detection so the arm returns to the inspect pose and tries again.
var errGripperEmpty = errors.New("gripper reported no object grabbed")

type armState string

const (
	stateIdle              armState = "idle"
	stateSearching         armState = "searching_for_objects"
	stateDetected          armState = "objects_detected"
	statePicking           armState = "picking"
	statePlacing           armState = "placing"
	stateWaitingForPartner armState = "waiting_for_partner"
	stateStopped           armState = "stopped"
	stateResetting         armState = "resetting"
)

// cyclePhase tracks which half of the cube-sorter cycle the worker is in. A
// cycle is sort-then-return. Phase persists across stop so the next start
// resumes the phase that was interrupted; a clean finish resets to sorting.
type cyclePhase string

const (
	phaseSorting   cyclePhase = "sorting"
	phaseReturning cyclePhase = "returning"
)

// convergeBarrier coordinates the arms' agreement that a phase is finished.
// Every round, each arm reports whether the source it is clearing came up empty
// on this pick; the round releases once all arms have reported, carrying
// whether the round was unanimously empty. A non-unanimous round — some arm
// still picked a block — sends every arm back for another pick-and-check, so the
// arms only leave a phase together, on a round where none of them found
// anything left.
//
// The sorter builds one barrier per `start` and pushes the same pointer onto
// each worker's cmdCh; it is reused for every round of every phase across the
// continuous self-loop, arming a fresh round each time one completes. leave()
// permanently drops a participant that has stopped or adopted a newer barrier,
// so the remaining arms aren't left waiting on a report that will never come.
type convergeBarrier struct {
	mu      sync.Mutex
	total   int  // participants still tied to this barrier
	arrived int  // reports counted toward the current round
	anyFull bool // did any arm report non-empty this round?
	cur     *convRound
}

// convRound is one reconciliation round. ready closes when every participant
// has reported; allEmpty is set just before the close and then never mutated,
// so every waiter reads a stable verdict for that round.
type convRound struct {
	ready    chan struct{}
	allEmpty bool
}

func newConvergeBarrier(participants int) *convergeBarrier {
	return &convergeBarrier{total: participants, cur: &convRound{ready: make(chan struct{})}}
}

// report records this arm's emptiness for the current round and returns that
// round. The arm that completes the round (last to report) finalizes the
// verdict — unanimously empty? — and arms a fresh round before returning.
func (b *convergeBarrier) report(empty bool) *convRound {
	b.mu.Lock()
	defer b.mu.Unlock()
	r := b.cur
	b.arrived++
	if !empty {
		b.anyFull = true
	}
	if b.arrived >= b.total {
		b.finishLocked(r)
	}
	return r
}

// leave permanently removes the caller from the barrier. Use when a worker
// won't report further — bailed out on a stop, or swapping to a fresh barrier
// from a new start. If the smaller quorum is already met by the arms that have
// reported, the current round is completed so any waiter unblocks instead of
// hanging.
func (b *convergeBarrier) leave() {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.total <= 0 {
		return
	}
	b.total--
	if b.total > 0 && b.arrived >= b.total {
		b.finishLocked(b.cur)
	}
}

// finishLocked resolves round r — recording whether it was unanimously empty —
// and arms a fresh round for the next pick. The caller must hold mu.
func (b *convergeBarrier) finishLocked(r *convRound) {
	r.allEmpty = !b.anyFull
	close(r.ready)
	b.arrived = 0
	b.anyFull = false
	b.cur = &convRound{ready: make(chan struct{})}
}

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
// from detection rather than discarding the block. The arm keeps retrying
// indefinitely — `stop` is the escape hatch if the obstruction is permanent.
// A brief backoff between attempts avoids hammering the system when stuck.
const cycleRestartBackoff = 2 * time.Second

// armWorker owns one arm and runs its pick-and-place cycle on a dedicated
// goroutine. Arm motion is planned on the gripper frame (so the frame system
// handles the gripper's mounting offset and finger geometry) and serialized via
// the shared motionMu so only one arm moves at a time. The worker mutex only
// guards in-memory state, so status and stop stay responsive.
type armWorker struct {
	name   string
	logger logging.Logger

	// deps
	arm        arm.Arm
	cam        camera.Camera
	gripper    gripper.Gripper
	segmenter  vision.Service
	startPose  toggleswitch.Switch
	zones      map[string]*zoneState
	returnArea *returnAreaState

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
	cmdCh     chan *convergeBarrier

	// interrupt control
	stopped  atomic.Bool
	opMu     sync.Mutex
	opCancel context.CancelFunc

	// in-memory state (short holds only)
	mu       sync.RWMutex
	state    armState
	phase    cyclePhase
	detected []DetectedObject
}

func (w *armWorker) gripperName() string {
	return w.gripper.Name().ShortName()
}

// run is the worker's event loop. A start command kicks off one runCycle,
// after which the worker self-loops on the SAME barrier so the per-round
// consensus keeps coordinating every cycle, not just the first. Transient
// failures (a failed pick, a stale-camera sensing error) back off and retry
// inside runCycle, so the loop only drops back to waiting on cmdCh when a stop
// interrupts it. If a new start lands in cmdCh between iterations, the worker
// leave()s the old barrier (releasing any partner still reconciling on it) and
// adopts the new one so both arms converge onto whichever barrier the most
// recent start published.
func (w *armWorker) run(wg *sync.WaitGroup) {
	defer wg.Done()
	for {
		select {
		case <-w.parentCtx.Done():
			return
		case b := <-w.cmdCh:
			for {
				if !w.runCycle(b) {
					break
				}
				if w.stopped.Load() {
					b.leave()
					break
				}
				select {
				case <-w.parentCtx.Done():
					b.leave()
					return
				case newB := <-w.cmdCh:
					b.leave()
					b = newB
				default:
				}
			}
		}
	}
}

// triggerWith requests a new cycle without blocking, handing the worker the
// barrier that reconciles it with its partner each round. Duplicate requests
// while a cycle is already running are dropped — the already-queued barrier
// takes effect for the next cycle.
func (w *armWorker) triggerWith(barrier *convergeBarrier) {
	select {
	case w.cmdCh <- barrier:
	default:
	}
}

// runCycle runs one sort→return cycle under the shared convergence barrier.
// Each phase advances only when every arm agrees its source is empty (see
// convergePhase), which also keeps the arms moving into the next phase together
// — so the barrier no longer needs a separate sort→return rendezvous. A worker
// resumed already in the returning phase skips the sort half. Returns true when
// a full sort→return cycle completes by consensus — the signal `run` uses to
// auto-loop — or false if a stop interrupts a phase, after leaving the barrier
// so the partner isn't left waiting on a participant that has dropped out.
func (w *armWorker) runCycle(barrier *convergeBarrier) bool {
	ctx := w.newOpCtx()
	w.stopped.Store(false)

	if w.currentPhase() == phaseSorting {
		if !w.convergePhase(ctx, barrier, phaseSorting) {
			barrier.leave()
			w.setState(stateIdle)
			return false
		}
		w.setPhase(phaseReturning)
	}

	if !w.convergePhase(ctx, barrier, phaseReturning) {
		barrier.leave()
		w.setState(stateIdle)
		return false
	}
	w.setPhase(phaseSorting)
	w.setState(stateIdle)
	return true
}

// convergePhase runs one phase as a sequence of single-pick rounds. Each round
// the arm picks at most one block from the source it is clearing, then
// reconciles with the other arm(s) at the barrier; the phase ends only on a
// round where every arm's source came up empty. So no arm leaves while another
// still has blocks, and — because each arm re-senses its source every round —
// a block that appears late, or a prior empty reading that was a transient
// miss, is caught before the arms move on. Transient pick/sense failures back
// off and retry the round; only a stop (interrupt) ends the phase early.
// Returns true when the phase completes by consensus, false on interrupt.
func (w *armWorker) convergePhase(ctx context.Context, barrier *convergeBarrier, phase cyclePhase) bool {
	dropHeld := true // first round sheds any block held over from a prior stop
	for {
		picked, err := w.phaseStep(ctx, phase, dropHeld)
		if err != nil {
			if w.interrupted(ctx, err) {
				w.logger.Infof("[%s] %s interrupted", w.name, phase)
				return false
			}
			// Transient failure (stale camera frame, blocked motion, missed
			// grab): back off and retry the round rather than abandoning the
			// phase. dropHeld on retry sheds a block a partial pick may have
			// left in the gripper.
			w.logger.Warnf("[%s] %s step failed; retrying after %v: %v", w.name, phase, cycleRestartBackoff, err)
			if serr := w.sleep(ctx, cycleRestartBackoff); serr != nil {
				return false
			}
			dropHeld = true
			continue
		}
		dropHeld = false

		allEmpty, err := w.reconcile(ctx, barrier, !picked, phase)
		if err != nil {
			return false
		}
		if allEmpty {
			return true
		}
	}
}

// reconcile reports this arm's emptiness for the current round and blocks until
// every arm has reported, returning whether the round was unanimously empty.
// Honors ctx/stop so a stop unblocks the wait.
func (w *armWorker) reconcile(ctx context.Context, barrier *convergeBarrier, empty bool, phase cyclePhase) (bool, error) {
	round := barrier.report(empty)
	select {
	case <-round.ready:
		return w.roundVerdict(ctx, round)
	default:
	}
	if empty {
		w.logger.Infof("[%s] %s: nothing left; waiting for other arm", w.name, phase)
		w.setState(stateWaitingForPartner)
	}
	select {
	case <-round.ready:
		return w.roundVerdict(ctx, round)
	case <-ctx.Done():
		return false, ctx.Err()
	}
}

// roundVerdict reads a completed round's outcome, mapping a stop that landed
// during the wait to an interrupt error.
func (w *armWorker) roundVerdict(ctx context.Context, round *convRound) (bool, error) {
	if err := w.checkInterrupt(ctx); err != nil {
		return false, err
	}
	return round.allEmpty, nil
}

// phaseStep performs one round's worth of work for the given phase: sense the
// source and pick at most one block. Returns picked=true if it moved a block,
// false if the source came up empty this round.
func (w *armWorker) phaseStep(ctx context.Context, phase cyclePhase, dropHeld bool) (bool, error) {
	if phase == phaseSorting {
		return w.sortStep(ctx, dropHeld)
	}
	return w.returnStep(ctx, dropHeld)
}

// sortStep senses owned blocks from the start pose and picks a single one into
// its zone. picked=false means the table held none of this arm's colors this
// round. dropHeld (first round) drops a block held over from a prior stop.
func (w *armWorker) sortStep(ctx context.Context, dropHeld bool) (bool, error) {
	if err := w.detectFromStart(ctx, dropHeld); err != nil {
		return false, err
	}
	label, ok := w.nextLabel()
	if !ok {
		return false, nil
	}
	if err := w.pickOne(ctx, label); err != nil {
		return false, err
	}
	return true, nil
}

// sortedZoneLabels returns this arm's zone labels in a stable order so the
// return phase scans zones deterministically from round to round.
func (w *armWorker) sortedZoneLabels() []string {
	labels := make([]string, 0, len(w.zones))
	for label := range w.zones {
		labels = append(labels, label)
	}
	slices.Sort(labels)
	return labels
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
	if err := w.liftDetected(ctx, label); err != nil {
		return err
	}
	w.setState(statePlacing)
	// Re-sense the zone before each placement so cell occupancy reflects
	// reality (other arm placed here, prior place was off, manual change).
	// Inspecting with the held block in view is fine — the segmenter ignores
	// it — and it saves a round trip versus inspecting before the lift.
	if err := w.prepareZone(ctx, label); err != nil {
		return err
	}
	if err := w.placeInZone(ctx, label); err != nil {
		return err
	}
	return w.setSwitch(ctx, w.startPose)
}

// liftDetected runs the pick path on the most recently detected object with
// the given label: open gripper, approach, descend, grab, lift. The caller is
// responsible for placing the held block (placeInZone for sort, placeOnTable
// for return).
func (w *armWorker) liftDetected(ctx context.Context, label string) error {
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
		w.logger.Warnf("[%s] gripper reported no object grabbed for %q; restarting from inspect pose", w.name, label)
		return errGripperEmpty
	}
	if err := w.sleep(ctx, 250*time.Millisecond); err != nil {
		return err
	}

	// Lift.
	liftPoint := pickPoint
	liftPoint.Z += liftHeightMm
	liftPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(liftPoint, &orient))
	return w.moveGripper(ctx, liftPose, nil, nil)
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

// inspectZoneByLabel drives the arm to the inspect pose for the zone matching
// label and runs a sense pass, leaving the arm parked there so the operator
// can view the camera stream. Useful for debugging placement.
func (w *armWorker) inspectZoneByLabel(label string) error {
	ctx := w.newOpCtx()
	w.stopped.Store(false)
	if err := w.prepareZone(ctx, label); err != nil {
		return err
	}
	w.setState(stateIdle)
	return nil
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
	if err := w.pickOne(ctx, label); err != nil {
		return err
	}
	w.removeDetected(label)
	w.setState(stateIdle)
	return nil
}

// stop cancels in-flight motion, halts the arm, and flips the silence flag so
// no further commands are issued until the next start.
func (w *armWorker) stop() error {
	w.stopped.Store(true)
	w.cancelOp()
	w.setState(stateStopped)

	ctx, cancel := context.WithTimeout(w.parentCtx, 5*time.Second)
	defer cancel()
	w.logger.Infof("[%s] stopping arm", w.name)
	return w.arm.Stop(ctx, nil)
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
	w.phase = phaseSorting
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

func (w *armWorker) currentPhase() cyclePhase {
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.phase
}

func (w *armWorker) setPhase(p cyclePhase) {
	w.mu.Lock()
	w.phase = p
	w.mu.Unlock()
	w.logger.Infof("[%s] phase -> %s", w.name, p)
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
