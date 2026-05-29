package cube_sorter

import (
	"fmt"

	"go.viam.com/rdk/services/motion"
)

// Zone describes where one block color is placed. Blocks are grid-packed into
// the zone, skipping cells that are already occupied (sensed via the inspect
// pose) so multiple blocks of a color don't pile onto the same spot.
type Zone struct {
	// Label is the detection label (color) this zone receives.
	Label string `json:"label"`
	// Origin is the world-frame XYZ (mm) of the zone center. When set, the
	// anchor visit is skipped and the gripper place orientation defaults to
	// straight down. Provide this when driving into the anchor pose is unsafe
	// (e.g. blocks may already be sitting at the grid corner). When Origin is
	// set, InspectPose may be omitted — the inspect pose is then derived as
	// InspectHeight directly above Origin with the gripper pointing down.
	Origin *[3]float64 `json:"origin,omitempty"`
	// InspectHeight is how far above Origin (mm) to position the gripper for
	// occupancy sensing. Only used when Origin is set and InspectPose is empty.
	// Defaults to defaultInspectHeightMm.
	InspectHeight float64 `json:"inspect_height,omitempty"`
	// AnchorPose is an arm-position-saver switch whose resulting gripper world
	// pose is the center of the zone (and the drop height/orientation). Its
	// world pose is captured once by driving to it and reading GetPose. Required
	// only when Origin is not set.
	AnchorPose string `json:"anchor_pose,omitempty"`
	// InspectPose is an arm-position-saver switch that puts the (eye-in-hand)
	// camera where it can see the zone, so occupied cells can be detected.
	// Required only when Origin is not set; otherwise the inspect pose is
	// derived from Origin + InspectHeight.
	InspectPose string `json:"inspect_pose,omitempty"`
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
			if z.AnchorPose == "" && z.Origin == nil {
				return nil, nil, fmt.Errorf("arm %d zone %q: either anchor_pose or origin must be set", i, z.Label)
			}
			if z.InspectPose == "" && z.Origin == nil {
				return nil, nil, fmt.Errorf("arm %d zone %q: inspect_pose is required unless origin is set", i, z.Label)
			}
			if z.Width <= 0 || z.Depth <= 0 {
				return nil, nil, fmt.Errorf("arm %d zone %q: width and depth must be positive", i, z.Label)
			}
			if z.AnchorPose != "" {
				deps = append(deps, z.AnchorPose)
			}
			if z.InspectPose != "" {
				deps = append(deps, z.InspectPose)
			}
		}
	}
	deps = append(deps, cfg.motionService())

	return deps, nil, nil
}
