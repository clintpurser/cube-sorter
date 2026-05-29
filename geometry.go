package cube_sorter

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

// minYawPoints is the floor below which a point cloud is too sparse to
// resolve a stable yaw.
const minYawPoints = 10

// yawAnisotropyThreshold is the minimum (worstArea / bestArea) across rotation
// candidates required to trust the recovered yaw. 1.0 = perfectly isotropic
// (circle), 2.0 = uniform square; 1.1 rejects circular blobs while accepting
// noisy real cubes.
const yawAnisotropyThreshold = 1.1

// yawSearchStepDeg is the resolution of the yaw search in (-45°, 45°]. 0.5°
// is well below the gripper alignment tolerance and keeps the search cheap.
const yawSearchStepDeg = 0.5

// approachHeightMm is how far above the grasp point the arm first moves before
// descending straight down onto the block.
const approachHeightMm = 100

// liftHeightMm is how far the arm raises after grasping, before transiting to
// the place pose.
const liftHeightMm = 150

// edgeYawDegrees finds the yaw (degrees, about world Z, in (-45°, 45°]) that
// aligns the cube's top edges with the world X/Y axes, by searching for the
// rotation that minimizes the axis-aligned bounding-box area of the XY-
// projected point cloud. At the correct yaw the bounding box hugs the cube's
// edges; any other yaw inscribes the cube in a strictly larger axis-aligned
// rectangle (worst case ≈ √2× per side at 45° off, i.e. 2× area).
//
// This silhouette-geometry approach is robust to the failure modes of moment-
// based estimators: PCA is rotation-invariant on 4-fold-symmetric shapes, and
// 4th-order moments suffer a sign ambiguity (square-like vs elongated) that
// flips the answer by 45° on noisy near-square silhouettes.
//
// Returns false if the cloud is too sparse or too isotropic (bbox area varies
// too little with rotation) to commit to an orientation.
func edgeYawDegrees(pc pointcloud.PointCloud) (float64, bool) {
	n := pc.Size()
	if n < minYawPoints {
		return 0, false
	}

	// Cache points so the rotation sweep doesn't pay the Iterate cost N times.
	points := make([]r3.Vector, 0, n)
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		points = append(points, p)
		return true
	})

	bestArea := math.Inf(1)
	worstArea := math.Inf(-1)
	var bestDeg float64
	for deg := -45.0; deg <= 45.0; deg += yawSearchStepDeg {
		rad := deg * math.Pi / 180
		cos := math.Cos(rad)
		sin := math.Sin(rad)

		xMin, yMin := math.Inf(1), math.Inf(1)
		xMax, yMax := math.Inf(-1), math.Inf(-1)
		for _, p := range points {
			// Rotate the point by -deg so the test axes "follow" the candidate yaw.
			rx := cos*p.X + sin*p.Y
			ry := -sin*p.X + cos*p.Y
			if rx < xMin {
				xMin = rx
			}
			if rx > xMax {
				xMax = rx
			}
			if ry < yMin {
				yMin = ry
			}
			if ry > yMax {
				yMax = ry
			}
		}
		area := (xMax - xMin) * (yMax - yMin)
		if area < bestArea {
			bestArea = area
			bestDeg = deg
		}
		if area > worstArea {
			worstArea = area
		}
	}

	if bestArea <= 0 {
		return 0, false
	}
	if worstArea/bestArea < yawAnisotropyThreshold {
		return 0, false
	}
	return bestDeg, true
}

// graspYawDegrees returns the gripper yaw (degrees) for a two-finger grasp:
// align the wrist with one of the cube's edges. The cube is 4-fold-symmetric
// so all four edges are equivalent grasps, and the result is normalized to
// (-90, 90] for the gripper's 180° symmetry. approachYaw is a fixed mounting
// or tuning offset, and used alone as the fallback when the cloud is too
// sparse or isotropic.
func graspYawDegrees(pc pointcloud.PointCloud, approachYaw float64) float64 {
	edge, ok := edgeYawDegrees(pc)
	if !ok {
		return normalizeYaw(approachYaw)
	}
	return normalizeYaw(edge + approachYaw)
}

// normalizeYaw folds an angle into (-90, 90] to exploit the gripper's 180°
// symmetry (the wrist can spin either way and grasp identically).
func normalizeYaw(deg float64) float64 {
	for deg > 90 {
		deg -= 180
	}
	for deg <= -90 {
		deg += 180
	}
	return deg
}
