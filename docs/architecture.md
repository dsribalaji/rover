# Architecture

Three independent entry points share one drivetrain idea: two PWM-enabled
gpiozero `Motor` channels, left and right, steered by differing their speeds.

```mermaid
flowchart LR
  CAM[Camera /dev/video0<br/>V4L2 640x480] -->|BGR frame| LOOP[person_follow.py main loop]
  YOLO[(yolov8n.pt<br/>imgsz 416)] -->|boxes| LOOP
  LOOP|steer left/right %| MOT[TwoChannelMotorController]
  MOT -->|GPIO 23/24/12| L[Left channel: motors 1 & 2]
  MOT -->|GPIO 27/22/13| R[Right channel: motors 3 & 4]
  KEY[raw stdin keystrokes] --> TP[rover.py KeyboardController] --> MOT
  MT[motor_test.py] --> L & R
```

## Module shapes

| File | Shape | Importable without side effects? |
| --- | --- | --- |
| `rover.py` | Two classes: `TwoChannelMotorController` (drivetrain verbs), `KeyboardController` (raw-key loop wrapping it); work happens under `if __name__ == "__main__"` | Yes |
| `motor_test.py` | Flat script; builds two `Motor` objects directly rather than reusing the controller class | No — drives the motors on import |
| `person_follow.py` | Config constants, model load, camera open, and robot construction all at module scope; the loop is a module-level `try/while` | No — loads YOLO, opens the camera, and grabs GPIO on import |
| `detect.py` | `main()` with argparse, plus two nested thread bodies and a signal handler | Yes (guarded) |

`person_follow.py` duplicates the motor controller class rather than importing it
from `rover.py`, and the duplicate is not identical: the follow version adds
`drive()` and a `GPIO_AVAILABLE` fallback, the teleop version adds pivots and
speed trim. That duplication is the main structural debt in the project — see
[code-notes.md](code-notes.md).

## GPIO_AVAILABLE fallback

In `person_follow.py`, the gpiozero import is wrapped in a `try`. If gpiozero is
missing, `TwoChannelMotorController.available` is `False`, every motor method
returns early, and the class prints once that commands are log-only. The rest of
the loop still runs, so the detection, steering maths, and state printing can be
exercised on a laptop with no GPIO. `rover.py` and `motor_test.py` import gpiozero
unguarded.

## The follow loop

Per iteration of the `while True` in `person_follow.py`:

1. `cap.read()` — blocking grab of one frame.
2. `frame_count += 1`; `if frame_count % SKIP_FRAMES != 0: continue`. With the
   committed `SKIP_FRAMES = 1` this is always `0`, i.e. every frame is processed.
   The knob only does something at 2 or higher.
3. Convert to gray, take the mean, fold into `ambient_light_ema` (seed 120.0,
   alpha 0.1).
4. If the light EMA is below `LOW_LIGHT_THRESHOLD` (72.0) and enhancement is on,
   the frame passed to the network is a CLAHE-boosted copy (details below).
5. At most once per `CAMERA_ADJUST_INTERVAL` (1.0 s), nudge the camera's own gain
   and exposure (`adapt_camera`).
6. Run YOLO with `conf=CONFIDENCE_THRESHOLD`, `imgsz=INPUT_SIZE`, `half=True`,
   `verbose=False`.
7. Among all returned boxes, keep the single largest one whose class name is
   `person` — area, not confidence, is the selection rule. Fold its confidence
   into `detect_conf_ema` (seed 0.50, alpha 0.15); with no detection, decay it
   toward zero by the same alpha.
8. Steer or search (next two sections).
9. Print `>>> ROBOT ACTION: …` only when the state string changes, and print an
   `FPS | Light | ConfEMA` line every 20 inferences.

## Steering maths

For a 640-wide frame, with `x1..y2` the chosen box:

```
person_center_x = (x1 + x2) / 2
center_offset   = person_center_x - 320
steer           = 0                       if |center_offset / 320| < STEER_DEADBAND (0.08)
                  clamp(center_offset / 320 * TURN_GAIN (0.75), -0.6, +0.6)   otherwise
left_speed      = clamp(FORWARD_SPEED (80) * (1 + steer), MIN_TRACK_SPEED (35), 100)
right_speed     = clamp(FORWARD_SPEED (80) * (1 - steer), MIN_TRACK_SPEED (35), 100)
```

Positive `steer` means the person sits right of centre, so the left wheel speeds
up and the right slows — the robot arcs right. `STEER_DIRECTION_INVERTED` flips
the sign if the chassis turns the wrong way.

The clamp bounds are what make it a follower rather than a turn-in-place rotator:
at maximum `steer = 0.6` the two sides would want 128 % and 32 %, and both are
clamped to 100 % and 35 %. The robot therefore always keeps both wheels driving
forward while following; it can never stop one side, back up, or pivot on the spot
in follow mode. Turning authority comes from that 100/35 ratio, not from reversing
a wheel.

Two more conditions short-circuit steering:

| Condition | Action | Printed state |
| --- | --- | --- |
| Box height > `TOO_CLOSE_HEIGHT` (600) | `stop_all()` | `stopped (too close)` |
| Offset inside the deadband, person near centre | both sides at `FORWARD_SPEED` | `following straight \| left=80% right=80%` |

Ultralytics returns boxes in the coordinates of the frame it was given, and the
camera is asked for 640×480, so a box can be at most ~480 px tall on a real
480-line stream — `TOO_CLOSE_HEIGHT = 600` is unreachable there and the proximity
stop stays off. That is intentional: the constant's own comment reads "High
threshold to prevent 'stopped (too close)' triggers". It does fire if the camera
ignores the resolution request and delivers a taller frame, which is the first
thing to check if the robot stops for no visible reason. To get a genuine
standoff, set it in the 300–450 range and walk the robot toward a person; the
historical values in `rover_learning.json` are `brake_height_px: 395` and
`stop_height_px: 440`, i.e. an earlier two-stage brake design.

## Lost-target search

When no `person` box is found, behaviour is a time-since-last-detection ladder over
`last_seen_direction` (the sign of the last non-zero `steer`):

| Time since last sighting | Condition | Action | Printed state |
| --- | --- | --- | --- |
| ≤ `SEARCH_HOLD_TIME` (1.5 s) | last direction known | scan that way at `SEARCH_SCAN_SPEED` (40) | `searching left/right (slow)` |
| ≤ `SEARCH_ALT_START_TIME` (5 s) | last direction known | same call, same speed — the two branches are behaviourally identical today | `searching left/right (extended slow)` |
| > 5 s | last direction known | flip `search_direction` every `SEARCH_ALT_INTERVAL` (6 s) and scan | `alternating search left/right (slow)` |
| any | never saw a person this run | `stop_all()` and reset the search bookkeeping | `stopped (no person)` |

`apply_search_scan()` calls `turn_left` / `turn_right`, which on the follow-side
controller means the inner channel runs **backward** at half speed (20 %) while
the outer runs forward at 40 % — a pivot, not an arc. This is the only place
autonomous motion reverses a wheel.

`last_seen_direction` starts at `0`, so a robot that has never detected a person
stands still until something enters the frame.

## Adaptive low light

Two independent mechanisms, one image-space and one sensor-space:

- **Image space** — `enhance_low_light()` converts BGR → YCrCb, applies
  `cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))` to the Y channel only, and
  converts back. Applied when `ambient_light_ema < 72.0` and
  `ENABLE_LOW_LIGHT_ENHANCE` is true. The original frame is untouched; only the
  copy fed to YOLO is boosted, and nothing is displayed.
- **Sensor space** — `adapt_camera()` runs at most once per second. If the scene is
  dark **and** `detect_conf_ema < 0.65`, it raises gain by 2.0 (range 0–64) and
  exposure by one step; if the scene is bright (`> 145.0`) it lowers both; in
  between it returns without touching the camera. Exposure ranges depend on the
  sign the driver reports: negative exposure values (the common V4L2 manual
  convention) clamp to −13…−1 with step 1, positive ones to 1…500 with step 5.
  A line prints only when the OS accepted at least one of the two settings, and
  the read-backs are NaN-guarded at startup (gain defaults to 0.0, exposure to −6.0).

## detect.py — the observation-only twin

`detect.py` exists to answer "is the camera pipeline fast enough, and what is the
network seeing", with no motion code at all. Its structure differs from the
follower in ways that matter for measurement:

```mermaid
flowchart LR
  CAP[_capture_loop thread<br/>read + resize to 640x360] -->|maxsize=1 queue| INF[_inference_loop thread<br/>YOLO at imgsz/conf]
  INF --> LOG[one line per frame:<br/>frame, size, EMA fps, class:count@conf]
  CAP -. queue full → drop oldest, keep newest .- INF
```

- Two daemon threads and a one-slot queue. When the queue is full the capture
  thread discards the frame waiting there and inserts the newest, so inference
  always sees the freshest image and latency does not pile up — the price is
  dropped frames, which is the correct trade for a follower.
- fps is an exponential moving average (`alpha = 0.15`) of 1 / inference-time, not
  a capture rate. It measures network throughput only.
- Detection output is aggregated per class as `name:count(@top_confidence)`, or
  `none`.
- Device selection is `cuda` when torch reports CUDA available, else `cpu`, with
  `model.model.half()` applied only on CUDA. This machine reports
  torch `2.11.0+cu130` with CUDA unavailable, so everything runs on the CPU.

## Interrupt and shutdown paths

| Script | Mechanism | Cleanup on exit |
| --- | --- | --- |
| `person_follow.py` | `signal.SIGINT` / `SIGTERM` → `shutdown()`; also `except KeyboardInterrupt` and a `finally` | stop motors, `close()` both channels, `cap.release()`, `sys.exit(0)` |
| `rover.py` | `try/except KeyboardInterrupt/finally` around the key loop | stop, `cleanup()` both channels |
| `detect.py` | `signal.SIGINT` handler prints and exits; `finally` in `main()` releases the camera | release camera, set the stop event, join both threads with a 1 s timeout |
| `motor_test.py` | `except Exception` (does **not** catch `KeyboardInterrupt`) then `finally: sys.exit()` | the `finally` turns Ctrl+C into a clean exit; the motors stop because gpiozero registers `atexit.register(_shutdown)` (verified in the installed `devices.py`, line 634), not because this script commands a stop |

None of them have a hardware kill switch or a heartbeat timeout — if the controlling
process is `SIGKILL`ed or the board loses power with the driver enabled, the last
PWM state is what the motor rail sees.
