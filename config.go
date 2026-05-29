package cube_sorter

import (
	"fmt"

	"go.viam.com/rdk/services/motion"
)

// Zone describes where one block color is placed. Blocks are grid-packed into
// the zone, skipping cells that are already occupied (sensed by hovering the
// camera above the zone) so multiple blocks of a color don't pile onto the
// same spot.
type Zone struct {
	// Label is the detection label (color) this zone receives.
	Label string `json:"label"`
	// Origin is the world-frame XYZ (mm) of the zone center. Drops use this
	// point as the grid origin and the gripper points straight down.
	Origin [3]float64 `json:"origin"`
	// InspectHeight is how far above Origin (mm) to position the gripper for
	// occupancy sensing. Defaults to defaultInspectHeightMm.
	InspectHeight float64 `json:"inspect_height,omitempty"`
	// InspectXOffset shifts the inspect pose along world X (mm) relative to
	// Origin so the camera can see the whole area instead of just the patch
	// directly below it. Defaults to defaultInspectXOffsetMm.
	InspectXOffset float64 `json:"inspect_x_offset,omitempty"`
	// Width (along world X) and Depth (along world Y) of the zone, in mm.
	Width float64 `json:"width"`
	Depth float64 `json:"depth"`
}

// ArmUnit describes one arm and the hardware/vision/zones that belong to it.
// Each unit owns the set of block colors named by its zones; an arm only ever
// picks objects whose detection label has a matching zone.
type ArmUnit struct {
	Arm       string `json:"arm_name"`
	Cam       string `json:"camera_name"`
	Gripper   string `json:"gripper_name"`
	Segmenter string `json:"segmenter_name"`
	StartPose string `json:"start_pose"`

	Zones []Zone `json:"zones"`

	// ReturnArea defines the table region this arm uses to place blocks when
	// returning them at the end of a sort cycle. Same shape as Zone (origin,
	// width, depth, inspect_height, inspect_x_offset), but no label —
	// placements are random within the bounds. Must lie inside the camera's
	// start-pose field of view, otherwise the next sort cycle won't detect
	// the returned blocks.
	ReturnArea Zone `json:"return_area"`

	// CubeHeight is the nominal block height (mm); the grasp descends
	// CubeHeight/2 below the visible top face to grab mid-block.
	CubeHeight float64 `json:"cube_height,omitempty"`
	// BlockSize is the block footprint (mm) used for grid cell pitch.
	// Defaults to CubeHeight.
	BlockSize float64 `json:"block_size,omitempty"`
	// Margin is the gap (mm) added between grid cells. Defaults to 0.
	Margin float64 `json:"margin,omitempty"`
	// GraspZOffset is an optional fine-tuning offset (mm) added to the grasp Z.
	GraspZOffset float64 `json:"grasp_z_offset,omitempty"`
	// ApproachYaw is a fixed yaw offset (degrees) added to the PCA-computed
	// grasp yaw, and the fallback yaw when the cloud is too sparse.
	ApproachYaw float64 `json:"approach_yaw,omitempty"`
}

// Config is the service config: one entry in `arms` per arm.
type Config struct {
	Arms   []ArmUnit `json:"arms"`
	Motion string    `json:"motion_service,omitempty"`
}

const defaultCubeHeight = 30.0

// defaultInspectHeightMm is how far above the zone origin the gripper is sent
// for occupancy sensing when InspectHeight is left unset.
const defaultInspectHeightMm = 200.0

// defaultInspectXOffsetMm shifts the inspect pose back along world X so the
// camera's FOV covers the whole zone instead of cropping the near edge.
const defaultInspectXOffsetMm = -100.0

// units returns the arm units with per-unit defaults applied. Viam decodes
// attributes via mapstructure (json tag names), so defaulting happens here.
func (cfg *Config) units() []ArmUnit {
	out := make([]ArmUnit, len(cfg.Arms))
	for i, u := range cfg.Arms {
		if u.CubeHeight == 0 {
			u.CubeHeight = defaultCubeHeight
		}
		if u.BlockSize == 0 {
			u.BlockSize = u.CubeHeight
		}
		out[i] = u
	}
	return out
}

func (cfg *Config) motionService() string {
	if cfg.Motion == "" {
		return motion.Named("builtin").String()
	}
	return cfg.Motion
}

// Validate ensures the config is valid and returns the implicit dependencies.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	units := cfg.units()
	if len(units) == 0 {
		return nil, nil, fmt.Errorf("must configure at least one arm (set `arms`)")
	}

	deps := []string{}
	for i, u := range units {
		if u.Arm == "" || u.Cam == "" || u.Gripper == "" || u.Segmenter == "" || u.StartPose == "" {
			return nil, nil, fmt.Errorf("arm %d: arm_name, camera_name, gripper_name, segmenter_name and start_pose are all required", i)
		}
		if len(u.Zones) == 0 {
			return nil, nil, fmt.Errorf("arm %d (%s): at least one zone is required (its label is a color this arm owns)", i, u.Arm)
		}
		deps = append(deps, u.Arm, u.Cam, u.Gripper, u.Segmenter, u.StartPose)
		for j, z := range u.Zones {
			if z.Label == "" {
				return nil, nil, fmt.Errorf("arm %d zone %d: label is required", i, j)
			}
			if z.Width <= 0 || z.Depth <= 0 {
				return nil, nil, fmt.Errorf("arm %d zone %q: width and depth must be positive", i, z.Label)
			}
		}
		if u.ReturnArea.Width <= 0 || u.ReturnArea.Depth <= 0 {
			return nil, nil, fmt.Errorf("arm %d (%s): return_area width and depth must be positive", i, u.Arm)
		}
	}
	deps = append(deps, cfg.motionService())

	return deps, nil, nil
}
