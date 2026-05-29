package cube_sorter

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// squareCloud builds a dense uniform-grid sample of a square (side 2·halfSide,
// centered at origin) rotated by yawDeg about Z. Used to drive edgeYawDegrees
// in tests with a known ground-truth orientation.
func squareCloud(t *testing.T, halfSide, yawDeg float64, gridSteps int) pointcloud.PointCloud {
	t.Helper()
	pc := pointcloud.NewBasicEmpty()
	c, s := math.Cos(yawDeg*math.Pi/180), math.Sin(yawDeg*math.Pi/180)
	step := 2 * halfSide / float64(gridSteps-1)
	for i := 0; i < gridSteps; i++ {
		for j := 0; j < gridSteps; j++ {
			x := -halfSide + float64(i)*step
			y := -halfSide + float64(j)*step
			rx := c*x - s*y
			ry := s*x + c*y
			if err := pc.Set(r3.Vector{X: rx, Y: ry, Z: 0}, nil); err != nil {
				t.Fatalf("pc.Set: %v", err)
			}
		}
	}
	return pc
}

func TestEdgeYawDegreesSquare(t *testing.T) {
	// 4-fold symmetry means any true yaw φ is equivalent to φ ± 90°·k; fold
	// expected and actual into (-45°, 45°] before comparing.
	fold := func(deg float64) float64 {
		for deg > 45 {
			deg -= 90
		}
		for deg <= -45 {
			deg += 90
		}
		return deg
	}
	for _, yaw := range []float64{0, 10, 30, -30, 44, 60, 89} {
		pc := squareCloud(t, 20, yaw, 21)
		got, ok := edgeYawDegrees(pc)
		if !ok {
			t.Errorf("yaw=%v: edgeYawDegrees returned ok=false", yaw)
			continue
		}
		if diff := math.Abs(fold(got - yaw)); diff > 0.5 {
			t.Errorf("yaw=%v: got %v (folded diff %v°)", yaw, got, diff)
		}
	}
}

// TestEdgeYawDegreesNoisyCube reproduces the regime that broke the 4th-moment
// estimator: a square silhouette with a slight asymmetric "tongue" of extra
// points on one side, mimicking partial visibility of a cube face. PCA can't
// see a 4-fold-symmetric shape at all and 4th moments flip the answer by 45°
// when the asymmetry tips Re(E[Z^4]) sign. The bbox search should still lock
// onto the dominant square edges.
func TestEdgeYawDegreesNoisyCube(t *testing.T) {
	fold := func(deg float64) float64 {
		for deg > 45 {
			deg -= 90
		}
		for deg <= -45 {
			deg += 90
		}
		return deg
	}
	for _, yaw := range []float64{0, 22.5, 30, -30, 44} {
		pc := squareCloud(t, 20, yaw, 21)
		// Add an asymmetric strip along the (rotated) +X side to mimic a partly-
		// visible cube face leaking into the silhouette.
		c, s := math.Cos(yaw*math.Pi/180), math.Sin(yaw*math.Pi/180)
		for i := 0; i < 5; i++ {
			for j := -5; j <= 5; j++ {
				x := 20.0 + float64(i)*1.0
				y := float64(j) * 1.0
				rx := c*x - s*y
				ry := s*x + c*y
				if err := pc.Set(r3.Vector{X: rx, Y: ry, Z: 0}, nil); err != nil {
					t.Fatalf("pc.Set: %v", err)
				}
			}
		}
		got, ok := edgeYawDegrees(pc)
		if !ok {
			t.Errorf("yaw=%v: edgeYawDegrees returned ok=false", yaw)
			continue
		}
		if diff := math.Abs(fold(got - yaw)); diff > 1.5 {
			t.Errorf("yaw=%v: got %v (folded diff %v°)", yaw, got, diff)
		}
	}
}

func TestEdgeYawDegreesIsotropicRejected(t *testing.T) {
	// A dense disc has no 4-fold structure; the anisotropy threshold should
	// reject it so callers fall back to approachYaw.
	pc := pointcloud.NewBasicEmpty()
	const r = 20.0
	for i := 0; i < 4000; i++ {
		theta := 2 * math.Pi * float64(i) / 4000
		// Fibonacci-ish radial fill to cover the disc roughly uniformly.
		rad := r * math.Sqrt(float64(i%200)/200.0)
		if err := pc.Set(r3.Vector{X: rad * math.Cos(theta), Y: rad * math.Sin(theta)}, nil); err != nil {
			t.Fatalf("pc.Set: %v", err)
		}
	}
	if _, ok := edgeYawDegrees(pc); ok {
		t.Error("expected isotropic disc to be rejected, got ok=true")
	}
}

func TestEdgeYawDegreesSparseRejected(t *testing.T) {
	pc := squareCloud(t, 20, 0, 3) // 9 points < minMomentPoints (10)
	if _, ok := edgeYawDegrees(pc); ok {
		t.Error("expected sparse cloud to be rejected, got ok=true")
	}
}

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
				Zones:      []Zone{{Label: "blue", Origin: [3]float64{100, 200, 30}, Width: 200, Depth: 200}},
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
				Zones: []Zone{{Label: "red", Origin: [3]float64{350, -120, 30}, Width: 200, Depth: 150}}},
		},
	}
	deps, _, err := good.Validate("")
	if err != nil {
		t.Fatalf("valid config rejected: %v", err)
	}
	for _, want := range []string{"a", "c", "g", "s", "h"} {
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
				Zones: []Zone{{Label: "red", Origin: [3]float64{0, 0, 0}, Width: 0, Depth: 100}}},
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
