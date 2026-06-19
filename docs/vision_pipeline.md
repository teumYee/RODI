# Vision and Escort Pipeline

## Goal

The current implementation follows the simple demo-first flow:

1. Patrol the mart with Nav2.
2. Detect people with YOLOv8.
3. Classify detected people as `child` or `adult`.
4. Treat a child without a nearby adult as an unguarded child.
5. Approach the child, speak a friendly guide message, escort to customer service, then resume patrol.

World and map geometry are owned by the navigation/environment side. This package avoids editing those files and uses launch parameters for runtime tuning.

## Launch

```bash
ros2 launch myrobot vision.launch.py alert_hold_sec:=10.0
```

Useful parameters:

- `rgb_topic`: RGB input for YOLO. Default: `/camera/overlay/image_raw`
- `classification_mode`: `bbox` or `depth`. Default: `bbox`
- `alert_hold_sec`: seconds a child must remain unguarded before alert.
- `alert_wander_prob`: probability that simulated adult guardians wander away.

## Child Detector

Node: `child_detector`

Inputs:

- RGB image: configurable by `rgb_topic`
- Depth image: `/camera/depth/image_raw`
- Depth camera info: `/camera/depth/camera_info`

Output:

- `/detected_persons` (`std_msgs/String`, JSON)

Payload:

```json
{
  "persons": [
    {
      "label": "child",
      "confidence": 0.82,
      "bbox": [120, 80, 210, 310],
      "depth_m": 2.34,
      "pos3d": [0.12, -0.04, 2.34],
      "class_reason": "bbox_h=230px aspect=2.56"
    }
  ],
  "stamp_sec": 123
}
```

Classification modes:

- `bbox`: demo-first mode. Uses bounding box height and height/width aspect ratio.
- `depth`: extension mode. Uses depth and camera intrinsics to estimate physical height.

## Guardian Logic

Node: `guardian_logic`

Input:

- `/detected_persons`

Output:

- `/child_alert` (`std_msgs/String`, JSON)

Payload:

```json
{
  "alert": true,
  "message": "보호자 없는 어린이 1명 감지",
  "child_count": 1,
  "unguarded_children": [
    {
      "pos3d": [0.12, -0.04, 2.34],
      "depth_m": 2.34
    }
  ]
}
```

Current logic:

- Filter detections by valid depth range.
- For each child, look for an adult within `guardian_dist_m`.
- If no adult is nearby for `alert_hold_sec`, publish `/child_alert`.

## Escort Response

Node: `child_response`

Inputs:

- `/child_alert`
- `/odom`
- `/gazebo/model_states`
- TF: `depth_camera_link` to `map`

Outputs:

- `/escort_status`: `busy` or `idle`
- `/escorting_child`: selected Gazebo child model name, or empty string when released
- `/escort_state`: explicit high-level state

States:

- `patrol`
- `detected`
- `approach`
- `escort`
- `wait_child`
- `arrived`
- `return`
- `idle`

The node uses Nav2 `navigate_to_pose` to approach the child and then drive to the customer service point. TTS is played by the shared `tts_utils.speak_ko()` helper.

## TTS

TTS uses `gTTS` with Korean text and plays generated MP3 through `mpg123`, falling back to `ffplay`.

The launch path currently relies on `child_response` for scenario-specific guidance messages. `tts_speaker` remains available as a standalone `/child_alert` warning speaker, but it is not part of `vision.launch.py` by default.

## Runtime Checks

Use these during integration:

```bash
ros2 topic echo /camera/depth/camera_info --once
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/overlay/image_raw
ros2 topic echo /detected_persons
ros2 topic echo /child_alert
ros2 topic echo /escort_state
```

Expected successful flow:

1. `/camera/overlay/image_raw` publishes camera frames with simulated people overlaid.
2. `/detected_persons` publishes detections with `label`, `bbox`, `depth_m`, and `pos3d`.
3. `/child_alert` fires after the configured hold time for an unguarded child.
4. `/escort_state` moves through `detected -> approach -> escort -> arrived -> return -> patrol`.
