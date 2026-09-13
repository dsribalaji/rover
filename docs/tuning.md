# Tuning

Every constant that affects behaviour, where it lives, what it does, and whether it
is actually wired up. Line numbers refer to the committed files as they stand with the
module docstrings added.

## `person_follow.py` — the follow controller

### Model and capture

| Constant | Line | Value | Effect | Live? |
| --- | --- | --- | --- | --- |
| `MODEL_PATH` | 98 | `yolov8n.pt` | weights file, resolved relative to the cwd | Yes |
| `CONFIDENCE_THRESHOLD` | 99 | `0.45` | per-box floor passed to YOLO; below it a person is simply not seen | Yes |
| `CAMERA_SOURCE` | 100 | `/dev/video0` | V4L2 node, with a numeric-index fallback to `0` | Yes |
| `INPUT_SIZE` | 104 | `416` | `imgsz` for inference; also set into `model.overrides` at load | Yes |
| `SKIP_FRAMES` | 105 | `1` | process every Nth frame; `frame_count % SKIP_FRAMES` | Yes, inert at `1` |

### Speed and steering

| Constant | Line | Value | Effect | Live? |
| --- | --- | --- | --- | --- |
| `FORWARD_SPEED` | 101 | `80` | base percent for both channels when tracking straight | Yes |
| `TURN_GAIN` | 111 | `0.75` | multiplies the normalised lateral offset to get `steer` | Yes |
| `STEER_DEADBAND` | 113 | `0.08` | offsets inside ±8 % of centre width produce `steer = 0` | Yes |
| `MIN_TRACK_SPEED` | 112 | `35` | floor for either channel while following — the outer wheel never stops | Yes |
| `STEER_DIRECTION_INVERTED` | 114 | `False` | flip to `True` if the robot steers away from the person | Yes |
| `TOO_CLOSE_HEIGHT` | 109 | `600` | box height above which the robot stops; unreachable on a true 480-line frame | Yes, effectively off |
| `CENTER_THRESHOLD` | 108 | `70` | superseded by `STEER_DEADBAND` + `TURN_GAIN` | **No** |
| `STEER_SPEED` | 110 | `90` | superseded by `FORWARD_SPEED` with the per-side clamps | **No** |

### Lost-target search

| Constant | Line | Value | Effect | Live? |
| --- | --- | --- | --- | --- |
| `SEARCH_HOLD_TIME` | 115 | `1.5` | first rung of the ladder; same action as the next rung today | Yes, no distinct effect |
| `SEARCH_ALT_START_TIME` | 117 | `5` | seconds after which the scan starts alternating direction | Yes |
| `SEARCH_ALT_INTERVAL` | 118 | `6` | seconds per scan direction once alternating | Yes |
| `SEARCH_SCAN_SPEED` | 119 | `40` | speed of the pivot scan (inner channel runs backward at 20) | Yes |
| `SEARCH_TURN_SPEED` | 116 | `60` | labelled legacy in its own comment | **No** |

### Adaptive low light

| Constant | Line | Value | Effect | Live? |
| --- | --- | --- | --- | --- |
| `ENABLE_LOW_LIGHT_ENHANCE` | 128 | `True` | apply CLAHE to the frame fed to YOLO when dark | Yes |
| `ADAPTIVE_LOW_LIGHT` | 122 | `True` | allow gain/exposure auto-tuning at all | Yes |
| `LOW_LIGHT_THRESHOLD` | 123 | `72.0` | gray-mean EMA below this counts as dark | Yes |
| `BRIGHT_LIGHT_THRESHOLD` | 124 | `145.0` | EMA above this counts as bright and pulls gain/exposure down | Yes |
| `LIGHT_EMA_ALPHA` | 125 | `0.1` | smoothing of the ambient-light estimate | Yes |
| `CONF_EMA_ALPHA` | 126 | `0.15` | smoothing of detection confidence; also the decay rate when nothing is seen | Yes |
| `CAMERA_ADJUST_INTERVAL` | 127 | `1.0` | minimum seconds between camera re-tunes | Yes |

Not exposed as constants, but part of the same system: `ambient_light_ema` seeds at
`120.0`, `detect_conf_ema` at `0.50`, gain range `0–64` step `2.0`, the
`detect_conf_ema < 0.65` gate inside `adapt_camera`, and the exposure fallback of
`-6.0` when the driver reports NaN at startup.

## `rover.py` — teleop

| Item | Value | Notes |
| --- | --- | --- |
| `self.speed` | `60` | starting percent for every move; the comment above it describes gpiozero's 0.0–1.0 unit instead |
| `increase_speed()` | +10, ceiling 100 | applies from the next keystroke |
| `decrease_speed()` | −10, floor 20 | 20 is the practical crawl floor |
| `turn_left` / `turn_right` | inner side `speed // 2` **backward**, outer `speed` forward | "gentle" only relative to the pivots |
| `pivot_left` / `pivot_right` | both sides at full `speed`, opposite directions | spin in place |

## `detect.py` — defaults

`--device 0`, `--conf 0.25`, `--imgsz 640`, `--width 640`, `--height 360`,
`--model yolov8n.pt`, fps EMA `alpha = 0.15`, queue depth 1.

## What `rover_learning.json` records

This file is a snapshot from an earlier build that tuned itself. No script in the
repo reads or writes it, so treat it as field notes, not configuration.

| Key | Value | Nearest current constant | Reading |
| --- | --- | --- | --- |
| `follow_speed` | 83 | `FORWARD_SPEED = 80` | tuned a little faster than the committed default |
| `brake_height_px` | 395 | `TOO_CLOSE_HEIGHT = 600` | a real standoff existed at ~395 px; 600 switched it off |
| `stop_height_px` | 440 | — (no two-stage brake now) | evidence of a brake-then-stop pair |
| `search_turn_speed` | 52 | `SEARCH_TURN_SPEED = 60` (dead), `SEARCH_SCAN_SPEED = 40` (live) | search pivots were faster before |
| `confidence_thr` | 0.49 | `CONFIDENCE_THRESHOLD = 0.45` | ran slightly stricter |
| `sessions` | 5 | — | five tuning sessions logged |
| `total_detections` | 2646 | — | cumulative person detections across those sessions |
| `total_lost_events` | 12 | — | times it lost a target it had |
| `total_close_bumps` | 23 | — | times it got too close — the event the current 600 px threshold no longer catches |

The two counters that matter for revisiting behaviour: 23 close bumps against a
`brake_height_px` of 395 argues that the proximity stop was doing real work and is
now disabled.

## Recipes

Change one constant at a time, and capture a log while you do it
(`./yolo_env/bin/python person_follow.py 2>&1 | tee session.log`). The lines worth
reading afterwards are `>>> ROBOT ACTION:` transitions (how often it flipped
left/right = wobble) and the periodic `FPS | Light | ConfEMA` line.

| Symptom | First move | Why |
| --- | --- | --- |
| Weaves left/right on a straight approach | raise `STEER_DEADBAND` toward `0.15`, and/or lower `TURN_GAIN` toward `0.4` | a bigger centre zone and a gentler proportional term stop the controller from chasing pixel noise |
| Turns too lazily, person slips out of frame | raise `TURN_GAIN` toward `1.0` and/or lower `MIN_TRACK_SPEED` | more differential authority; slower inner wheel means a tighter arc |
| Cannot turn hard enough before giving up | lower `MIN_TRACK_SPEED` toward 20 | the inner wheel is what limits turn radius, since follow mode never reverses it |
| Steers away from the person | set `STEER_DIRECTION_INVERTED = True` | the left/right channel assignment is mirrored relative to the camera |
| Stops dead when a person is close | that is `TOO_CLOSE_HEIGHT`; set it to the standoff you want (300–450) or leave 600 to disable | box height is a proxy for distance |
| Loses people in dim rooms | confirm `ENABLE_LOW_LIGHT_ENHANCE`; raise `LOW_LIGHT_THRESHOLD` toward 90; raise `CONFIDENCE_THRESHOLD` only if false positives appear | the CLAHE path and the auto-gate both key off the light EMA |
| Detection flickers at the edge of confidence | lower `CONFIDENCE_THRESHOLD` toward `0.35` and watch `ConfEMA` | more raw hits, more jitter; the EMA is the tiebreaker |
| Search spins uselessly when nobody was ever there | nothing to change — `last_seen_direction` starts at `0`, so it stands still until first detection | by design |
| Inference too slow | keep `INPUT_SIZE` at 416 or drop to 320, and set `SKIP_FRAMES = 2` | both cut work per frame; skipping only does anything at 2 or above |

`INPUT_SIZE` trades accuracy for latency directly: 416 was chosen over `detect.py`'s
640 default. Compare the two with `detect.py --imgsz 416` and `--imgsz 640` before
touching the follower.
