package cube_sorter

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func TestNormalizeYaw(t *testing.T) {
	cases := []struct {
		in, want float64
	}{
		{0, 0},
		{45, 45},
		{90, 90},
		{-90, 90},
		{135, -45},
		{-135, 45},
		{180, 0},
		{270, 90},
	}
	for _, c := range cases {
		if got := normalizeYaw(c.in); math.Abs(got-c.want) > 1e-9 {
			t.Errorf("normalizeYaw(%v) = %v, want %v", c.in, got, c.want)
		}
	}
}

func TestConfigUnitsDefaults(t *testing.T) {
	cfg := &Config{
		Arms: []ArmUnit{
			{Arm: "a", Cam: "c", Gripper: "g", Segmenter: "s", StartPose: "h",
				Zones:      []Zone{{Label: "blue", AnchorPose: "blue-anchor", InspectPose: "blue-inspect", Width: 200, Depth: 200}},
				CubeHeight: 40},
		},
	}
	units := cfg.units()
	if len(units) != 1 {
		t.Fatalf("expected 1 unit, got %d", len(units))
	}
	if units[0].BlockSize != 40 {
		t.Errorf("block_size should default to cube_height = 40, got %v", units[0].BlockSize)
	}
}

func TestValidate(t *testing.T) {
	good := &Config{
		Arms: []ArmUnit{
			{Arm: "a", Cam: "c", Gripper: "g", Segmenter: "s", StartPose: "h",
				Zones: []Zone{{Label: "red", AnchorPose: "red-anchor", InspectPose: "red-inspect", Width: 200, Depth: 150}}},
		},
	}
	deps, _, err := good.Validate("")
	if err != nil {
		t.Fatalf("valid config rejected: %v", err)
	}
	for _, want := range []string{"a", "c", "g", "s", "h", "red-anchor", "red-inspect"} {
		if !contains(deps, want) {
			t.Errorf("deps missing %q: %v", want, deps)
		}
	}

	noZones := &Config{
		Arms: []ArmUnit{{Arm: "a", Cam: "c", Gripper: "g", Segmenter: "s", StartPose: "h"}},
	}
	if _, _, err := noZones.Validate(""); err == nil {
		t.Error("expected error for arm with no zones")
	}

	badZone := &Config{
		Arms: []ArmUnit{
			{Arm: "a", Cam: "c", Gripper: "g", Segmenter: "s", StartPose: "h",
				Zones: []Zone{{Label: "red", AnchorPose: "x", InspectPose: "y", Width: 0, Depth: 100}}},
		},
	}
	if _, _, err := badZone.Validate(""); err == nil {
		t.Error("expected error for zone with non-positive width")
	}

	empty := &Config{}
	if _, _, err := empty.Validate(""); err == nil {
		t.Error("expected error for config with no arms")
	}
}

func TestBuildCellsGridPacking(t *testing.T) {
	z := &zoneState{
		cfg:    Zone{Width: 100, Depth: 100},
		pitch:  50,
		origin: spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 10}),
	}
	z.buildCells()
	if len(z.cells) != 4 {
		t.Fatalf("expected 2x2 = 4 cells, got %d", len(z.cells))
	}
	// All cells share the origin Z and are unoccupied.
	for _, c := range z.cells {
		if c.occupied {
			t.Error("new cell should be unoccupied")
		}
		if math.Abs(c.pt.Z-10) > 1e-9 {
			t.Errorf("cell Z = %v, want 10", c.pt.Z)
		}
	}

	// nextFree returns a cell, and after marking it the next call returns another.
	c1, ok := z.nextFree()
	if !ok {
		t.Fatal("expected a free cell")
	}
	c1.occupied = true
	c2, ok := z.nextFree()
	if !ok || c2 == c1 {
		t.Error("expected a different free cell after occupying the first")
	}

	// nearestCell maps a point near a cell center back to that cell.
	if got := z.nearestCell(c1.pt.X, c1.pt.Y); got != c1 {
		t.Error("nearestCell did not return the matching cell")
	}
	if got := z.nearestCell(1000, 1000); got != nil {
		t.Error("nearestCell should return nil for a far point")
	}
}

func contains(s []string, v string) bool {
	for _, x := range s {
		if x == v {
			return true
		}
	}
	return false
}
