package cube_sorter

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

// minPCAPoints is the floor below which a point cloud is considered too sparse
// to estimate a reliable principal axis.
const minPCAPoints = 10

// approachHeightMm is how far above the grasp point the arm first moves before
// descending straight down onto the block.
const approachHeightMm = 100

// liftHeightMm is how far the arm raises after grasping, before transiting to
// the place pose.
const liftHeightMm = 150

// principalYawDegrees estimates the yaw (degrees, about world Z) of a block's
// long axis by running a 2D PCA on the point cloud projected to the table
// plane. Returns false if the cloud is too sparse or degenerate to be useful.
func principalYawDegrees(pc pointcloud.PointCloud) (float64, bool) {
	n := pc.Size()
	if n < minPCAPoints {
		return 0, false
	}

	var sumX, sumY float64
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		sumX += p.X
		sumY += p.Y
		return true
	})
	meanX := sumX / float64(n)
	meanY := sumY / float64(n)

	var sxx, sxy, syy float64
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		dx := p.X - meanX
		dy := p.Y - meanY
		sxx += dx * dx
		sxy += dx * dy
		syy += dy * dy
		return true
	})

	// Degenerate (no spread) -> no meaningful axis.
	if sxx+syy < 1e-9 {
		return 0, false
	}

	// Angle of the principal (max-variance) eigenvector of [[sxx,sxy],[sxy,syy]].
	angleRad := 0.5 * math.Atan2(2*sxy, sxx-syy)
	return angleRad * 180 / math.Pi, true
}

// graspYawDegrees returns the gripper yaw (degrees) for a two-finger grasp:
// the fingers close perpendicular to the block's long axis. A cube is
// 90°-symmetric, so the result is normalized into (-90, 90]. approachYaw is
// added as a fixed mounting/tuning offset, and used alone as the fallback when
// PCA cannot find an axis.
func graspYawDegrees(pc pointcloud.PointCloud, approachYaw float64) float64 {
	principal, ok := principalYawDegrees(pc)
	if !ok {
		return normalizeYaw(approachYaw)
	}
	return normalizeYaw(principal + 90 + approachYaw)
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
