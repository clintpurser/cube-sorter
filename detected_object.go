package cube_sorter

import (
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/objectdetection"
)

type DetectedObject struct {
	Label string
	// Object is the point cloud in CAMERA frame as returned by the segmenter.
	// Don't transform this to world later — the camera will have moved between
	// detection and pick, and TransformPointCloud uses the frame system as it
	// stands at the call. Use WorldObject for any world-frame math instead.
	Object    viz.Object
	Detection objectdetection.Detection
	// WorldPC is Object's point cloud transformed into the world frame at
	// detection time, while the camera was still at its detection pose. This
	// is what the pick path uses for grasp geometry.
	WorldPC pointcloud.PointCloud
	// WorldCenter is the point-cloud center in world frame (== WorldPC's
	// MetaData().Center()), surfaced separately for the get_detected_objects
	// response.
	WorldCenter r3.Vector
}

func (do DetectedObject) Serialize() map[string]any {
	boundingBox := do.Detection.BoundingBox()
	result := map[string]any{
		"label": do.Label,
		"box": map[string]any{
			"xMin": boundingBox.Min.X,
			"yMin": boundingBox.Min.Y,
			"xMax": boundingBox.Max.X,
			"yMax": boundingBox.Max.Y,
		},
		"world_center_mm": map[string]any{
			"x": do.WorldCenter.X,
			"y": do.WorldCenter.Y,
			"z": do.WorldCenter.Z,
		},
	}
	return result
}
