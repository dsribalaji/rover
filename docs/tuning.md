# Tuning

Every constant that affects behaviour, where it lives, what it does, and whether it
is actually wired up. Knobs are named by their CONFIG constant or by the function that
reads them. There are deliberately no line numbers in this file: they rot on the first
edit.

## `person_follow.py` — the follow controller

Everything below sits in the `# ===== CONFIG =====` block at the top of the file. Setup
happens inside `main()` behind `if __name__ == "__main__"`, so importing the module no
longer loads the model, opens the camera or claims GPIO.

### Model and capture

| Constant | Value | Effect | Live? |
| --- | --- | --- | --- |
| `MODEL_PATH` | `yolov8n.pt` | weights file, resolved relative to the cwd; no longer tracked by git, see [setup.md](setup.md#model-weights) | Yes |
| `CONFIDENCE_THRESHOLD` | `0.45` | per-box floor passed to YOLO; below it a person is simply not seen | Yes |
| `CAMERA_SOURCE` | `/dev/video0` | V4L2 node, opened with `cv2.CAP_V4L2` in `open_camera`, with a numeric-index fallback to `0` | Yes |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `640` / `480` | capture size requested in `open_camera`; these were literals there until now | Yes |
| `INPUT_SIZE` | `416` | `imgsz` for inference; also set into `model.overrides` at load | Yes |
| `FRAME_SKIP` | `0` | `0` = infer on every frame, N = infer the first frame and then every (N+1)th (`(frame_count - 1) % (FRAME_SKIP + 1)`). Replaces `SKIP_FRAMES`, where `0` raised `ZeroDivisionError` on the first frame | Yes |
| `MOTOR_WATCHDOG_S` | `0.75` | how long the `motor_watchdog` thread tolerates silence before `robot.stop_all()`. The loop beats it once per processed frame, including when a camera read fails, and it prints one `>>> WATCHDOG:` line per stall | Yes |

`MOTOR_WATCHDOG_S` has to stay above the time one control-loop iteration takes, or the
watchdog stops the drivetrain between commands — that is the constraint written above the
constant. [code-notes.md](code-notes.md#6-performance) records 5.0 inferences/s on the Pi,
a median of 200 ms with p95 at 250 ms, and 0.75 s is about three times that p95. Every
skipped frame widens the gap between beats, so raising `FRAME_SKIP` means raising this
too.

### Speed and steering

| Constant | Value | Effect | Live? |
| --- | --- | --- | --- |
| `FORWARD_SPEED` | `80` | base percent for both channels when tracking straight | Yes |
| `TURN_GAIN` | `0.75` | multiplies the normalised lateral offset to get `steer`, clamped to ±0.6 | Yes |
| `STEER_DEADBAND` | `0.08` | offsets inside ±8 % of half the frame width (±26 px at 640 wide) produce `steer = 0` | Yes |
| `MIN_TRACK_SPEED` | `35` | floor for either channel while following — the outer wheel never stops | Yes |
| `STEER_DIRECTION_INVERTED` | `False` | flip to `True` if the robot steers away from the person | Yes |
| `MOUNT_OFFSET_PX` | `0` | camera yaw bias in px, subtracted from the lateral offset in `follow_person()`. Set it to the mean `Offset` a person standing on the chassis centreline produces, so a camera that looks right of the centreline takes a negative value | Yes |
| `TOO_CLOSE_HEIGHT_RATIO` | `0.99` | proximity stop as a share of the frame's row count: the box must exceed 475 of the 480 rows it captures | Yes, effectively off |

The offset is measured against the frame the camera actually handed back
(`frame_width / 2`), not an assumed 640, which is what makes `MOUNT_OFFSET_PX` mean the
same thing at any capture size. A camera bolted to a chassis is never perfectly square to
it, and a constant lateral bias is a mounting error rather than a steering error, so it
gets its own knob instead of being folded into `TURN_GAIN`. The 2026-09-13 bench run
logged a mean `Offset` of −73 px with the person standing near the centreline, so with
`MOUNT_OFFSET_PX = -73` the controller treats that as straight ahead. It is the knob to
re-read after any remount, before touching `TURN_GAIN` or `STEER_DEADBAND`. The ratio
form of the proximity stop is explained under
[rover_learning.json](#what-rover_learningjson-records).

`CENTER_THRESHOLD`, `STEER_SPEED` and `SEARCH_TURN_SPEED` are gone from the file. They
were the three rows this table used to carry a **No** for; they are physically deleted
rather than left behind waiting for a cleanup pass, so every constant listed above is
read by the loop.

### Lost-target search

| Constant | Value | Effect | Live? |
| --- | --- | --- | --- |
| `SEARCH_HOLD_TIME` | `1.5` | first rung of the ladder; same action as the next rung today, only the printed phase differs | Yes, no distinct effect |
| `SEARCH_ALT_START_TIME` | `5` | seconds after which the scan starts alternating direction | Yes |
| `SEARCH_ALT_INTERVAL` | `6` | seconds per scan direction once alternating | Yes |
| `SEARCH_SCAN_SPEED` | `40` | speed of the pivot scan (inner channel runs backward at 20) | Yes |

### Adaptive low light

| Constant | Value | Effect | Live? |
| --- | --- | --- | --- |
| `ENABLE_LOW_LIGHT_ENHANCE` | `True` | apply CLAHE to the frame fed to YOLO when dark | Yes |
| `ADAPTIVE_LOW_LIGHT` | `True` | allow gain/exposure auto-tuning at all | Yes |
| `LOW_LIGHT_THRESHOLD` | `72.0` | gray-mean EMA below this counts as dark | Yes |
| `BRIGHT_LIGHT_THRESHOLD` | `145.0` | EMA above this counts as bright and pulls gain/exposure down | Yes |
| `LIGHT_EMA_ALPHA` | `0.1` | smoothing of the ambient-light estimate | Yes |
| `CONF_EMA_ALPHA` | `0.15` | smoothing of detection confidence; also the decay rate when nothing is seen | Yes |
| `CAMERA_ADJUST_INTERVAL` | `1.0` | minimum seconds between camera re-tunes | Yes |

Not exposed as constants, but part of the same system: `ambient_light_ema` seeds at
`120.0`, `detect_conf_ema` at `0.50`, gain range `0–64` step `2.0`, the
`detect_conf_ema < 0.65` gate inside `adapt_camera`, and the exposure fallback of
`-6.0` when the driver reports NaN at startup. The per-side ceiling while following is
the literal `100` in `follow_person`, not `MAX_SPEED` from `drivetrain.py`. The inference
call itself is now plain `model(frame, verbose=False, conf=..., imgsz=INPUT_SIZE)`: the
`half=True` argument is gone, since this CPU-only torch build accepted it and did nothing
with it.

## `drivetrain.py` — the shared motor class behind `rover.py` teleop

The pins and this class used to be written out twice, once in `rover.py` and once in
`person_follow.py`. Both copies are gone; `rover.py`, `person_follow.py` and
`motor_test.py` import the one in `drivetrain.py`.

| Item | Value | Notes |
| --- | --- | --- |
| Pin constants | `LEFT_FORWARD 23` / `LEFT_BACKWARD 24` / `LEFT_ENABLE 12`, `RIGHT_FORWARD 27` / `RIGHT_BACKWARD 22` / `RIGHT_ENABLE 13` | BCM numbers, the only copy in the repo; the header-pin table is in [hardware.md](hardware.md) |
| `self.speed` | `60` | starting percent for every move; the comment above it that described gpiozero's 0.0–1.0 unit did not survive the move |
| `drive(left_pct, right_pct)` | positive forward, negative reverse, `0` stop | one method now serves teleop and the follower; the follow copy used to stop a side on any non-positive value |
| `increase_speed()` | +`SPEED_STEP` (10), ceiling `MAX_SPEED` (100) | applies from the next keystroke |
| `decrease_speed()` | −`SPEED_STEP` (10), floor `MIN_SPEED` (20) | 20 is the practical crawl floor |
| `turn_left` / `turn_right` | inner side `speed // 2` **backward**, outer `speed` forward | "gentle" only relative to the pivots |
| `pivot_left` / `pivot_right` | both sides at full `speed`, opposite directions | spin in place |

## `detect.py` — defaults

`--device /dev/video0` (a bare index such as `0` works too; V4L2 is pinned, so index N
means `/dev/videoN` rather than OpenCV's probe order), `--conf 0.25`,
`--imgsz 416`, `--width 640`, `--height 480`, `--model yolov8n.pt`, fps EMA
`alpha = 0.15`, queue depth 1. The capture is opened with `cv2.CAP_V4L2`. `--interval`
and `--skip` are no longer in the parser at all — they were accepted and never read.

## What `rover_learning.json` records

This file is a snapshot from an earlier build that tuned itself. No script in the
repo reads or writes it, so treat it as field notes, not configuration.

| Key | Value | Nearest current constant | Reading |
| --- | --- | --- | --- |
| `follow_speed` | 83 | `FORWARD_SPEED = 80` | tuned a little faster than the committed default |
| `brake_height_px` | 395 | `TOO_CLOSE_HEIGHT_RATIO` ≈ `0.82` of a 480-row frame | a real standoff existed at ~395 px; the old absolute `TOO_CLOSE_HEIGHT = 600` switched it off |
| `stop_height_px` | 440 | ≈ `0.92` of 480 rows, and no two-stage brake now | evidence of a brake-then-stop pair |
| `search_turn_speed` | 52 | `SEARCH_SCAN_SPEED = 40` (live) | search pivots were faster before; the dead `SEARCH_TURN_SPEED = 60` it used to be compared with is deleted |
| `confidence_thr` | 0.49 | `CONFIDENCE_THRESHOLD = 0.45` | ran slightly stricter |
| `sessions` | 5 | — | five tuning sessions logged |
| `total_detections` | 2646 | — | cumulative person detections across those sessions |
| `total_lost_events` | 12 | — | times it lost a target it had |
| `total_close_bumps` | 23 | — | times it got too close — neither the old 600 px threshold nor today's `0.99` catches these at the current mount |

The two counters that matter for revisiting behaviour: 23 close bumps against a
`brake_height_px` of 395 argues that the proximity stop was doing real work.

Why the ratio form: a pixel height is only meaningful at one capture resolution, and
600 px could never be reached on a 480-row frame, which is exactly how the stop went
silently dead. A share of the row count says "the person fills the frame" and means the
same thing at 360, 480 or 720 rows, so changing `CAMERA_HEIGHT` no longer moves the
standoff by accident. The two recorded heights convert at 480 rows, which is what
`CAMERA_HEIGHT` asks for: 395/480 = 0.82 and 440/480 = 0.92.

Where `0.99` sits today: the bench run behind the CONFIG comment measured box heights
with a median of 459 rows and a maximum of 478 at the current low, upward-tilted mount,
and 0.99 × 480 = 475. The gate is therefore above almost every box the mount produces and
just below the largest one, so it fires only at the last moment before contact and the
23 close bumps in `rover_learning.json` mostly go uncatched. Tighten toward `0.85` after
the camera comes back higher and tilted down.

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
| Drifts to one side even with the person standing dead centre | with `MOUNT_OFFSET_PX = 0`, stand a person on the centreline, read the `Offset=` they produce, set `MOUNT_OFFSET_PX` to it | a camera yaw that is not parallel to the chassis is a mounting error; no steering gain absorbs a constant bias |
| Stops dead when a person is close | that is `TOO_CLOSE_HEIGHT_RATIO`; the old 395 px standoff is about `0.82` at 480 rows, and `0.99` leaves it disabled | box height is a proxy for distance |
| Loses people in dim rooms | confirm `ENABLE_LOW_LIGHT_ENHANCE`; raise `LOW_LIGHT_THRESHOLD` toward 90; raise `CONFIDENCE_THRESHOLD` only if false positives appear | the CLAHE path and the auto-gate both key off the light EMA |
| Detection flickers at the edge of confidence | lower `CONFIDENCE_THRESHOLD` toward `0.35` and watch `ConfEMA` | more raw hits, more jitter; the EMA is the tiebreaker |
| Search spins uselessly when nobody was ever there | nothing to change — `last_seen_direction` starts at `0`, so it stands still until first detection | by design |
| Motors keep stopping mid-approach | raise `MOTOR_WATCHDOG_S` toward `1.5` before touching anything else | the watchdog, not the planner, is stopping the drivetrain; the loop is not beating it often enough |
| Inference too slow | keep `INPUT_SIZE` at 416 or drop to 320, and set `FRAME_SKIP = 1` to infer every other frame | both cut work per frame; skipping also widens the gap between watchdog beats, so raise `MOTOR_WATCHDOG_S` with it |

`INPUT_SIZE` trades accuracy for latency directly: 416 was chosen over `detect.py`'s
old 640 default. `detect.py` now defaults to 416 as well, so compare the two with an
explicit `detect.py --imgsz 640` before touching the follower.
