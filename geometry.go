package cube_sorter

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

// minMomentPoints is the floor below which a point cloud is considered too
// sparse to estimate stable 4th-order central moments.
const minMomentPoints = 10

// squareAnisotropyThreshold is the minimum value of |E[Z^4]| / E[|Z|^2]^2
// (where Z = X + iY centered) required to trust the recovered edge angle.
// That ratio is 0 for a perfect circle and ≈ 0.6 for a perfect uniform square;
// 0.1 rejects 4-fold-isotropic blobs while still accepting noisy real cubes.
const squareAnisotropyThreshold = 0.1

// approachHeightMm is how far above the grasp point the arm first moves before
// descending straight down onto the block.
const approachHeightMm = 100

// liftHeightMm is how far the arm raises after grasping, before transiting to
// the place pose.
const liftHeightMm = 150

// edgeYawDegrees estimates the yaw (degrees, about world Z) of one of a cube
// top face's edges from its XY-projected point cloud. 2nd-order moments (PCA)
// are rotation-invariant for any 4-fold-symmetric shape, so they return noise
// on a cube; the 4th-order central moments carry the 4-fold structure and
// resolve a well-defined edge angle in (-45°, 45°]. Returns false if the cloud
// is too sparse or too 4-fold-isotropic (circular / noisy) to trust.
//
// Note: this is calibrated for square-like silhouettes (where Re(E[Z^4]) < 0
// in the canonical frame). Strongly elongated rectangles flip that sign and
// would come back rotated 45° off — fine for cubes, not a generic edge finder.
func edgeYawDegrees(pc pointcloud.PointCloud) (float64, bool) {
	n := pc.Size()
	if n < minMomentPoints {
		return 0, false
	}
	nf := float64(n)

	var sumX, sumY float64
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		sumX += p.X
		sumY += p.Y
		return true
	})
	meanX := sumX / nf
	meanY := sumY / nf

	var s20, s02 float64
	var s40, s04, s22, s31, s13 float64
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		dx := p.X - meanX
		dy := p.Y - meanY
		dx2 := dx * dx
		dy2 := dy * dy
		s20 += dx2
		s02 += dy2
		s40 += dx2 * dx2
		s04 += dy2 * dy2
		s22 += dx2 * dy2
		s31 += dx2 * dx * dy
		s13 += dx * dy * dy2
		return true
	})

	// Real and imaginary parts of E[(X+iY)^4]. For a square aligned with the
	// world axes, im = 0 and re = -4/15·a^4 < 0; under a rotation by φ this
	// pair traces a circle, multiplied by e^{4iφ}.
	re := (s40 - 6*s22 + s04) / nf
	im := 4 * (s31 - s13) / nf
	r2 := (s20 + s02) / nf

	if r2 < 1e-9 {
		return 0, false
	}
	// Require |E[Z^4]| ≥ threshold · E[|Z|^2]^2 so we don't read a yaw out of
	// pure noise on a near-circular cloud.
	if re*re+im*im < squareAnisotropyThreshold*squareAnisotropyThreshold*r2*r2*r2*r2 {
		return 0, false
	}

	// E[Z^4] = c · e^{4iφ} with c < 0 (square-like), so arg(E[Z^4]) = π + 4φ
	// and φ = arg(-E[Z^4]) / 4.
	angleRad := 0.25 * math.Atan2(-im, -re)
	return angleRad * 180 / math.Pi, true
}

// graspYawDegrees returns the gripper yaw (degrees) for a two-finger grasp:
// align the wrist with one of the cube's edges. The cube is 4-fold-symmetric
// so all four edges are equivalent grasps, and the result is normalized to
// (-90, 90] for the gripper's 180° symmetry. approachYaw is a fixed mounting
// or tuning offset, and used alone as the fallback when the cloud is too
// sparse or 4-fold-isotropic.
func graspYawDegrees(pc pointcloud.PointCloud, approachYaw float64) float64 {
	edge, ok := edgeYawDegrees(pc)
	if !ok {
		return normalizeYaw(approachYaw)
	}
	return normalizeYaw(edge + approachYaw + 45)
}

// normalizeYaw folds an angle into (-90, 90] to exploit a cube's 90° symmetry.
func normalizeYaw(deg float64) float64 {
	for deg > 90 {
		deg -= 180
	}
	for deg <= -90 {
		deg += 180
	}
	return deg
}
