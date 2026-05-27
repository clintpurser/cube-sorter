# Cube Sorter Robot

A pick and place demo built on the Viam platform using computer vision and motion planning to sort
colored blocks. It supports **one or more arms**, each with its own gripper and camera, sorting the
colors it owns. Motion is **serialized** so only one arm moves at a time (collision-free by
construction), and the routine is **interruptible** — call `stop` to halt mid-motion and hand the
arms to a teleoperation module, then `resume` to re-detect and continue.

## Features

- Computer vision-based object detection using segmentation
- Coordinate frame transformations (camera frame to world frame)
- Motion planning with constraints for precise arm control
- Two-finger gripper grasping with PCA-based yaw alignment to the block
- Multiple arms, each owning a set of block colors; one arm moves at a time
- Interruptible: `stop` / `resume` for clean teleoperation hand-off

## Prerequisites

- Go 1.16 or higher
- Access to a Viam robot instance with, per arm:
  - A robot arm (e.g. uFactory Lite6)
  - A depth camera (e.g. RealSense)
  - A two-finger gripper
  - A vision service that supports both `GetObjectPointClouds` and `DetectionsFromCamera`,
    emitting labels that match that arm's zone labels

## Model clint:cube-sorter:cube-sorter

The business logic service for sorting colored blocks into bins, or other arbitrary pick and place
tasks.

### Configuration

Configure an `arms` array — one entry per arm. The following attribute template can be used:

```json
{
  "arms": [
    {
      "arm_name": <string>,
      "camera_name": <string>,
      "gripper_name": <string>,
      "segmenter_name": <string>,
      "start_pose": <string>,
      "zones": [
        {
          "label": <string>,
          "anchor_pose": <string>,
          "inspect_pose": <string>,
          "width": <number>,
          "depth": <number>
        }
      ],
      "cube_height": <number>,
      "block_size": <number>,
      "margin": <number>,
      "grasp_z_offset": <number>,
      "approach_yaw": <number>
    }
  ],
  "motion_service": <string>
}
```

> **Gripper frame.** Motion is planned on the **gripper** component, not the arm, so the gripper's
> mounting offset and finger geometry are taken from the Viam frame system. Configure your gripper
> component's `frame` (translation + geometry) so its origin sits at the grasp point between the
> fingertips — then no manual length offset is needed.

#### Per-arm attributes (entries in `arms`)

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `arm_name` | string  | Required  | Name of the arm component (used for `Stop` and as the kinematic chain). |
| `camera_name` | string | Required  | Name of the camera component providing point cloud data. |
| `gripper_name` | string | Required  | Name of the two-finger gripper component; motion is planned on its frame. |
| `segmenter_name` | string | Required  | Name of the vision service providing point cloud objects + detections. |
| `start_pose` | string | Required  | Name of the `arm-position-saver` switch component providing the start position. |
| `zones` | array | Required  | One zone per color **this arm owns** (see below). An arm only picks labels with a matching zone. |
| `cube_height` | number | Optional | Nominal block height (mm); the grasp descends `cube_height / 2` below the visible top to grab mid-block. Default `30`. |
| `block_size` | number | Optional | Block footprint (mm) used for grid cell pitch. Default `cube_height`. |
| `margin` | number | Optional | Gap (mm) added between grid cells. Default `0`. |
| `grasp_z_offset` | number | Optional | Fine-tuning offset (mm) added to the grasp Z. Default `0`. |
| `approach_yaw` | number | Optional | Yaw offset (degrees) added to the PCA-computed grasp yaw, and the fallback yaw when the point cloud is too sparse. Default `0`. |

#### Zone attributes (entries in a unit's `zones`)

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `label` | string | Required | Detection label (color) this zone receives. |
| `anchor_pose` | string | Required | `arm-position-saver` switch whose resulting gripper world pose is the **center** of the zone (and the drop height/orientation). Captured once by driving to it. |
| `inspect_pose` | string | Required | `arm-position-saver` switch that points the (eye-in-hand) camera at the zone so occupied cells can be detected before placing. |
| `width` | number | Required | Zone extent along world X (mm). |
| `depth` | number | Required | Zone extent along world Y (mm). |

Blocks are grid-packed into the zone (cell pitch = `block_size + margin`). At the start of each cycle
the arm visits each `inspect_pose` and marks occupied cells, so blocks of the same color don't pile
onto each other and pre-existing blocks are avoided.

#### Top-level attributes

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `motion_service` | string | Optional  | Name of the motion service to use for planning. Defaults to `"builtin"`. |

#### Example Configuration (two arms)

```json
{
  "arms": [
    {
      "arm_name": "left-arm",
      "camera_name": "left-cam",
      "gripper_name": "left-gripper",
      "segmenter_name": "left-segmenter",
      "start_pose": "left-home-pose",
      "cube_height": 30,
      "margin": 10,
      "zones": [
        {"label": "red_cube", "anchor_pose": "red-zone-center", "inspect_pose": "red-zone-inspect", "width": 200, "depth": 150},
        {"label": "yellow_cube", "anchor_pose": "yellow-zone-center", "inspect_pose": "yellow-zone-inspect", "width": 200, "depth": 150}
      ]
    },
    {
      "arm_name": "right-arm",
      "camera_name": "right-cam",
      "gripper_name": "right-gripper",
      "segmenter_name": "right-segmenter",
      "start_pose": "right-home-pose",
      "cube_height": 30,
      "margin": 10,
      "zones": [
        {"label": "green_cube", "anchor_pose": "green-zone-center", "inspect_pose": "green-zone-inspect", "width": 200, "depth": 150},
        {"label": "blue_cube", "anchor_pose": "blue-zone-center", "inspect_pose": "blue-zone-inspect", "width": 200, "depth": 150}
      ]
    }
  ]
}
```

### DoCommand

#### Start

Begin the sort routine on **every** arm in the background and return immediately. Each arm moves to
its start pose, detects its owned colors, and sorts them. Arms move one at a time. Poll `get_status`
to track progress.

```json
{
  "command": "start"
}
```

Returns:
```json
{
    "success": <boolean>,
    "status": "started"
}
```

#### Stop

Immediately cancel any in-flight motion and stop all arms, then issue **no further arm commands** so
a teleoperation module can take over. Returns right away.

```json
{
  "command": "stop"
}
```

Returns:
```json
{
    "success": <boolean>
}
```

#### Resume

Clear the stopped state and continue: each arm returns to its start pose, opens its gripper (to drop
any block it was holding when stopped), re-detects, and sorts whatever blocks remain. Robust to the
arms/blocks having been moved during teleoperation.

```json
{
  "command": "resume"
}
```

Returns:
```json
{
    "success": <boolean>,
    "status": "resumed"
}
```

#### Get Status

Query per-arm status (`idle` -> `searching_for_objects` -> `objects_detected` -> `picking` ->
`placing` -> `idle`, or `stopped` / `resetting`) and detected objects. Always returns promptly, even
mid-pick.

```json
{
  "command": "get_status"
}
```

Returns:
```json
{
    "success": <boolean>,
    "arms": {
        "<arm_name>": {
            "status": <string>,
            "detected_objects": <{"label": <string>, "box": <{"xMin": <number>, "yMin": <number>, "xMax": <number>, "yMax": <number>}>}[]>
        }
    }
}
```

#### Reset

Stop any current action, open the gripper, and return each arm to its start position.

```json
{
  "command": "reset"
}
```

Returns:
```json
{
    "success": <boolean>
}
```

#### Get Detected Objects

Run a detection pass on every arm and return the pickable objects found per arm.

```json
{
  "command": "get_detected_objects"
}
```

Returns:
```json
{
    "success": <boolean>,
    "objects": {
        "<arm_name>": <{"label": <string>, "box": <{"xMin": <number>, "yMin": <number>, "xMax": <number>, "yMax": <number>}>}[]>
    }
}
```

#### Pick Object

Pass the label of a pickable object to perform a single pick-and-place. The arm that owns that color
(i.e. has a zone with that label) performs the pick.

```json
{
    "command": "pick_object",
    "label": <string>
}
```

Returns:
```json
{
    "success": <boolean>
}
```

## Development

### 1. Clone the Repository

```bash
git clone <repository-url>
cd cube-sorter
```

### 2. Install Dependencies

```bash
go mod tidy
```

See the [Developer Guide](./DEVELOPER_GUIDE.md) for more information about building and running the module.


## How It Works

Each arm runs its own cycle on a background worker; a shared lock ensures only one arm moves at a
time, so plans are collision-free by construction.

1. **Initialization**: Connects to the Viam robot and builds one worker per configured arm.
2. **Home Position**: The arm moves to its start pose.
3. **Object Detection**: The arm's camera + vision service detect objects; only the colors that arm
   owns (its zone labels) are tracked.
4. **Coordinate Transformation**: Detected object coordinates are transformed from camera frame to
   world frame.
5. **Zone Sensing**: For each owned color, the arm visits the zone's inspect pose and marks which
   grid cells are already occupied.
6. **Grasp Planning**: Motion is planned on the gripper frame. The grasp Z descends `cube_height/2`
   below the visible top so the open fingers straddle the block, and a yaw is computed by running
   PCA on the object's point cloud so the fingers close across the block's narrow axis.
7. **Pick**: The arm approaches from above, descends straight down under a linear constraint, and
   closes the two-finger gripper.
8. **Place**: Lifts the block and drops it into the next free grid cell of its color's zone.
9. **Return Home**: Returns the arm to its start pose and continues with the next block.

At any point, `stop` cancels in-flight motion and halts the arms so a teleoperation module can take
over; `resume` re-detects and continues.

## Safety Notes

- Ensure the workspace is clear before running
- Emergency stop should be readily accessible on the physical robot
- After `stop`, the module issues no arm commands until `resume`, so a teleoperation module can
  safely own the arms

## License

[Apache 2.0](./LICENSE)
