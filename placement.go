package cube_sorter

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

type zoneCell struct {
	pt       r3.Vector
	occupied bool
}

// zoneState holds the runtime state for one color's placement zone.
type zoneState struct {
	cfg     Zone
	anchor  toggleswitch.Switch
	inspect toggleswitch.Switch
	pitch   float64

	origin spatialmath.Pose // captured anchor world pose; nil until captured
	cells  []*zoneCell
}

// buildCells lays an axis-aligned grid (along world X/Y) centered on the zone
// origin, with cells sized to the configured pitch.
func (z *zoneState) buildCells() {
	origin := z.origin.Point()
	cols := int(z.cfg.Width / z.pitch)
	rows := int(z.cfg.Depth / z.pitch)
	if cols < 1 {
		cols = 1
	}
	if rows < 1 {
		rows = 1
	}

	spanX := float64(cols) * z.pitch
	spanY := float64(rows) * z.pitch
	z.cells = make([]*zoneCell, 0, cols*rows)
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			z.cells = append(z.cells, &zoneCell{pt: r3.Vector{
				X: origin.X - spanX/2 + z.pitch*(float64(c)+0.5),
				Y: origin.Y - spanY/2 + z.pitch*(float64(r)+0.5),
				Z: origin.Z,
			}})
		}
	}
}

// nearestCell returns the cell whose center is within half a pitch of (x,y).
func (z *zoneState) nearestCell(x, y float64) *zoneCell {
	half := z.pitch / 2
	for _, cell := range z.cells {
		if abs(cell.pt.X-x) <= half && abs(cell.pt.Y-y) <= half {
			return cell
		}
	}
	return nil
}

func (z *zoneState) nextFree() (*zoneCell, bool) {
	for _, cell := range z.cells {
		if !cell.occupied {
			return cell, true
		}
	}
	return nil, false
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}

// prepareZones (re)senses every owned zone while the arm is empty: it captures
// each anchor's world pose once, builds the grid, then drives to the inspect
// pose and marks occupied cells. Safe to call at the start of every cycle.
func (w *armWorker) prepareZones(ctx context.Context) error {
	for label := range w.zones {
		if err := w.prepareZone(ctx, label); err != nil {
			return err
		}
	}
	return nil
}

func (w *armWorker) prepareZone(ctx context.Context, label string) error {
	z, ok := w.zones[label]
	if !ok {
		return fmt.Errorf("no zone configured for %q", label)
	}

	if z.origin == nil {
		if err := w.setSwitch(ctx, z.anchor); err != nil {
			return err
		}
		pose, err := w.motion.GetPose(ctx, w.gripperName(), "world", nil, nil)
		if err != nil {
			return err
		}
		z.origin = pose.Pose()
		z.buildCells()
		w.logger.Infof("[%s] zone %q anchored at %v with %d cells", w.name, label, z.origin.Point(), len(z.cells))
	}

	return w.senseZone(ctx, z)
}

// senseZone drives to the inspect pose and marks cells occupied by detected
// blocks. Must be called with an empty gripper so the camera isn't occluded.
func (w *armWorker) senseZone(ctx context.Context, z *zoneState) error {
	if err := w.setSwitch(ctx, z.inspect); err != nil {
		return err
	}
	if err := w.sleep(ctx, 500*time.Millisecond); err != nil { // settle
		return err
	}

	objs, err := w.segmenter.GetObjectPointClouds(ctx, "", nil)
	if err != nil {
		return err
	}

	for _, cell := range z.cells {
		cell.occupied = false
	}

	occupied := 0
	for _, obj := range objs {
		inWorld, err := w.client.TransformPointCloud(ctx, obj, w.cam.Name().ShortName(), "world")
		if err != nil {
			return err
		}
		md := inWorld.MetaData()
		center := md.Center()
		if cell := z.nearestCell(center.X, center.Y); cell != nil && !cell.occupied {
			cell.occupied = true
			occupied++
		}
	}
	w.logger.Infof("[%s] zone %q: %d of %d cells occupied", w.name, z.cfg.Label, occupied, len(z.cells))
	return nil
}

// placeInZone moves a held block into the next free cell of its zone and
// releases it. Assumes the zone has already been prepared this cycle.
func (w *armWorker) placeInZone(ctx context.Context, label string) error {
	z, ok := w.zones[label]
	if !ok {
		return fmt.Errorf("no zone configured for %q", label)
	}
	if z.origin == nil {
		if err := w.prepareZone(ctx, label); err != nil {
			return err
		}
	}

	cell, ok := z.nextFree()
	if !ok {
		return fmt.Errorf("zone %q is full", label)
	}

	orient := z.origin.Orientation()
	above := cell.pt
	above.Z += approachHeightMm

	w.logger.Infof("[%s] placing %q at cell %v", w.name, label, cell.pt)

	abovePose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(above, orient))
	dropPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(cell.pt, orient))

	if err := w.moveGripper(ctx, abovePose, nil, nil); err != nil {
		return err
	}
	if err := w.moveGripper(ctx, dropPose, &pickConstraints, nil); err != nil {
		return err
	}
	if err := w.gripper.Open(ctx, nil); err != nil {
		return err
	}
	if err := w.sleep(ctx, 250*time.Millisecond); err != nil {
		return err
	}
	if err := w.moveGripper(ctx, abovePose, nil, nil); err != nil {
		return err
	}

	cell.occupied = true
	return nil
}
