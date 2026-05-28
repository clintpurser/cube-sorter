package cube_sorter

import (
	"github.com/golang/geo/r3"

	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/objectdetection"
)

type DetectedObject struct {
	Label     string
	Object    viz.Object
	Detection objectdetection.Detection
	// WorldCenter is the point-cloud center transformed from the camera frame
	// into the world frame at detection time. Stashed here so callers (and
	// `get_detected_objects`) can see where the module thinks the object is
	// without needing to attempt a pick.
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
