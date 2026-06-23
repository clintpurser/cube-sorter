package cube_sorter

import (
	"context"
	"fmt"
	"math/rand"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// sampleAttempts caps how many random candidates sampleReturnPosition tries
// before giving up. Generous for a reasonably sized area.
const sampleAttempts = 50

// placeRetryAttempts caps how many fresh samples placeWithRetry tries when
// placeOnTable fails (typically due to a kinematically unreachable goal at the
// edge of the return area). A new random sample usually lands in a reachable
// spot.
const placeRetryAttempts = 5

// returnAreaState holds the runtime state for the table region an arm uses to
// place blocks when returning them. Unlike zoneState, placements are
// continuous (random within bounds), not grid-quantized.
type returnAreaState struct {
	cfg   Zone
	pitch float64

	origin spatialmath.Pose // built lazily on first use

	// placed accumulates XY positions during the current return phase so we
	// avoid colliding with our own recent placements even if sensing missed
	// one. Reset at the start of each return phase (returnStep's first round).
	placed []r3.Vector
	// sensed is the most recent set of XY centers detected in the area;
	// refreshed by senseReturnArea before each placement.
	sensed []r3.Vector
}

func (r *returnAreaState) ensureOrigin() {
	if r.origin != nil {
		return
	}
	pt := r3.Vector{X: r.cfg.Origin[0], Y: r.cfg.Origin[1], Z: r.cfg.Origin[2]}
	r.origin = spatialmath.NewPose(pt, &spatialmath.OrientationVectorDegrees{OZ: -1})
}

// returnStep performs one round of the return phase: it scans the arm's zones
// (stable order) for a block to bring back to the table and, on finding one,
// lifts it and places it at a free spot in the return area. picked=true means
// it moved a block; picked=false means every zone scanned clean — the empty
// signal convergePhase reconciles with the other arm. dropHeld (first round)
// sheds a block held over from a prior stop and resets the placed-position
// memory for the new phase.
func (w *armWorker) returnStep(ctx context.Context, dropHeld bool) (bool, error) {
	w.returnArea.ensureOrigin()

	if dropHeld {
		w.returnArea.placed = nil
		if err := w.setSwitch(ctx, w.startPose); err != nil {
			return false, err
		}
		w.logger.Infof("[%s] return: dropping any held block at start", w.name)
		if err := w.gripper.Open(ctx, nil); err != nil {
			return false, err
		}
		if err := w.sleep(ctx, 250*time.Millisecond); err != nil {
			return false, err
		}
	}

	for _, label := range w.sortedZoneLabels() {
		z := w.zones[label]
		ensureZoneOrigin(z)
		if err := w.checkInterrupt(ctx); err != nil {
			return false, err
		}
		dets, err := w.detectInZone(ctx, z)
		if err != nil {
			return false, err
		}
		if len(dets) == 0 {
			continue
		}

		// Found one. Stash it where liftDetected can find it, bring exactly one
		// block back, then yield to the reconcile — the next round re-scans.
		w.mu.Lock()
		w.detected = []DetectedObject{dets[0]}
		w.mu.Unlock()

		if err := w.liftDetected(ctx, label); err != nil {
			return false, err
		}
		if err := w.senseReturnArea(ctx); err != nil {
			return false, err
		}
		w.setState(statePlacing)
		if err := w.placeWithRetry(ctx); err != nil {
			return false, err
		}
		return true, nil
	}

	return false, nil
}

// ensureZoneOrigin lazily builds a zone's origin pose + grid cells. Mirrors
// the lazy-init block in prepareZone but without the senseZone call, since
// the return phase doesn't depend on cell occupancy.
func ensureZoneOrigin(z *zoneState) {
	if z.origin != nil {
		return
	}
	pt := r3.Vector{X: z.cfg.Origin[0], Y: z.cfg.Origin[1], Z: z.cfg.Origin[2]}
	z.origin = spatialmath.NewPose(pt, &spatialmath.OrientationVectorDegrees{OZ: -1})
	z.buildCells()
}

// detectInZone hovers the camera over a zone and returns world-frame
// detections matching the zone's label. Mirrors detectFromStart but with the
// camera over the zone instead of the start pose, so the return phase grasps
// based on freshly observed block centers rather than remembered grid cells.
func (w *armWorker) detectInZone(ctx context.Context, z *zoneState) ([]DetectedObject, error) {
	origin := z.origin.Point()
	height := z.cfg.InspectHeight
	if height == 0 {
		height = defaultInspectHeightMm
	}
	xOffset := z.cfg.InspectXOffset
	if xOffset == 0 {
		xOffset = defaultInspectXOffsetMm
	}
	inspectPt := r3.Vector{X: origin.X + xOffset, Y: origin.Y, Z: origin.Z + height}
	inspectPose := referenceframe.NewPoseInFrame("world",
		spatialmath.NewPose(inspectPt, &spatialmath.OrientationVectorDegrees{OZ: -1}))
	if err := w.moveGripper(ctx, inspectPose, nil, nil); err != nil {
		return nil, err
	}
	if err := w.sleep(ctx, 500*time.Millisecond); err != nil {
		return nil, err
	}

	objs, err := w.segmenter.GetObjectPointClouds(ctx, "", nil)
	if err != nil {
		return nil, err
	}
	dets, err := w.segmenter.DetectionsFromCamera(ctx, "", nil)
	if err != nil {
		return nil, err
	}

	out := []DetectedObject{}
	for _, obj := range objs {
		if obj.Geometry.Label() != z.cfg.Label {
			continue
		}
		for _, det := range dets {
			if det.Label() != z.cfg.Label {
				continue
			}
			selected := *obj
			inWorld, terr := w.client.TransformPointCloud(ctx, &selected, w.cam.Name().ShortName(), "world")
			if terr != nil {
				w.logger.Warnf("[%s] return: transform %q failed: %v", w.name, z.cfg.Label, terr)
				break
			}
			md := inWorld.MetaData()
			out = append(out, DetectedObject{
				Label:       z.cfg.Label,
				Object:      selected,
				Detection:   det,
				WorldPC:     inWorld,
				WorldCenter: md.Center(),
			})
			break
		}
	}
	w.logger.Infof("[%s] return: detected %d %q in zone", w.name, len(out), z.cfg.Label)
	return out, nil
}

// senseReturnArea hovers the camera above the return area and records every
// detected block's world-frame XY center. Called before each placement so the
// sampler has a fresh view of obstacles.
func (w *armWorker) senseReturnArea(ctx context.Context) error {
	r := w.returnArea
	origin := r.origin.Point()
	height := r.cfg.InspectHeight
	if height == 0 {
		height = defaultInspectHeightMm
	}
	xOffset := r.cfg.InspectXOffset
	if xOffset == 0 {
		xOffset = defaultInspectXOffsetMm
	}
	inspectPt := r3.Vector{X: origin.X + xOffset, Y: origin.Y, Z: origin.Z + height}
	inspectPose := referenceframe.NewPoseInFrame("world",
		spatialmath.NewPose(inspectPt, &spatialmath.OrientationVectorDegrees{OZ: -1}))
	if err := w.moveGripper(ctx, inspectPose, nil, nil); err != nil {
		return err
	}
	if err := w.sleep(ctx, 500*time.Millisecond); err != nil {
		return err
	}

	objs, err := w.segmenter.GetObjectPointClouds(ctx, "", nil)
	if err != nil {
		return err
	}

	r.sensed = r.sensed[:0]
	for _, obj := range objs {
		inWorld, terr := w.client.TransformPointCloud(ctx, obj, w.cam.Name().ShortName(), "world")
		if terr != nil {
			w.logger.Warnf("[%s] return area: transform failed: %v", w.name, terr)
			continue
		}
		md := inWorld.MetaData()
		r.sensed = append(r.sensed, md.Center())
	}
	w.logger.Infof("[%s] return area: sensed %d block(s)", w.name, len(r.sensed))
	return nil
}

// sampleReturnPosition picks a uniform random XY within the return area whose
// distance from every sensed-or-placed block is at least one pitch. Z is the
// configured drop Z (origin.Z). Returns an error if no valid spot is found
// within sampleAttempts tries — caller should log and stop returning rather
// than wedge the arm.
func (w *armWorker) sampleReturnPosition() (r3.Vector, error) {
	r := w.returnArea
	origin := r.origin.Point()
	halfW := r.cfg.Width / 2
	halfD := r.cfg.Depth / 2
	minDist := r.pitch

	for attempt := 0; attempt < sampleAttempts; attempt++ {
		candidate := r3.Vector{
			X: origin.X - halfW + rand.Float64()*r.cfg.Width,
			Y: origin.Y - halfD + rand.Float64()*r.cfg.Depth,
			Z: origin.Z,
		}
		if farFromAll(candidate, r.sensed, minDist) && farFromAll(candidate, r.placed, minDist) {
			return candidate, nil
		}
	}
	return r3.Vector{}, fmt.Errorf("no free spot in return area after %d attempts", sampleAttempts)
}

func farFromAll(pt r3.Vector, others []r3.Vector, minDist float64) bool {
	min2 := minDist * minDist
	for _, o := range others {
		dx := pt.X - o.X
		dy := pt.Y - o.Y
		if dx*dx+dy*dy < min2 {
			return false
		}
	}
	return true
}

// placeWithRetry samples a return-area position and places the held block,
// retrying with fresh samples when placeOnTable fails. IK "unreachable" errors
// at the edge of the area are the motivating case — a new random sample
// almost always lands in a reachable spot. Re-sensing is skipped because the
// table state is unchanged between attempts.
func (w *armWorker) placeWithRetry(ctx context.Context) error {
	var lastErr error
	for attempt := 1; attempt <= placeRetryAttempts; attempt++ {
		pt, err := w.sampleReturnPosition()
		if err != nil {
			return err
		}
		if err := w.placeOnTable(ctx, pt); err != nil {
			if w.interrupted(ctx, err) {
				return err
			}
			w.logger.Warnf("[%s] return: place at (%.1f, %.1f, %.1f) failed (attempt %d/%d): %v",
				w.name, pt.X, pt.Y, pt.Z, attempt, placeRetryAttempts, err)
			lastErr = err
			continue
		}
		return nil
	}
	return fmt.Errorf("return: place failed after %d attempts: %w", placeRetryAttempts, lastErr)
}

// placeOnTable moves a held block to a position in the return area and
// releases. Mirrors placeInZone but uses the random sampled point instead of
// a grid cell, and appends to placed on success.
func (w *armWorker) placeOnTable(ctx context.Context, pt r3.Vector) error {
	orient := w.returnArea.origin.Orientation()
	above := pt
	above.Z += approachHeightMm

	w.logger.Infof("[%s] return: placing at %v (drop Z=%.1f, approach Z=%.1f)",
		w.name, pt, pt.Z, above.Z)

	abovePose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(above, orient))
	dropPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(pt, orient))

	if err := w.moveGripper(ctx, abovePose, nil, nil); err != nil {
		return fmt.Errorf("return approach %v: %w", above, err)
	}
	if err := w.moveGripper(ctx, dropPose, &pickConstraints, nil); err != nil {
		return fmt.Errorf("return descent %v: %w", pt, err)
	}
	if err := w.gripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("return open: %w", err)
	}
	if err := w.sleep(ctx, 250*time.Millisecond); err != nil {
		return err
	}
	if err := w.moveGripper(ctx, abovePose, nil, nil); err != nil {
		return fmt.Errorf("return retreat %v: %w", above, err)
	}

	w.returnArea.placed = append(w.returnArea.placed, pt)
	return nil
}
