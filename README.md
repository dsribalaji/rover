# Rover

A person-following rover built on a Raspberry Pi 5. A camera feed is run through
YOLOv8n, the largest detected `person` box is turned into left/right wheel speeds,
and those drive a four-motor skid-steer chassis through two GPIOzero motor
channels. The same drivetrain code doubles as a keyboard teleop and a bench test.

The project is six files at the repository root. There is no package structure,
build step, or service — each file is run directly with the project's Python
interpreter.

| File | Role | Moves the robot | Needs a camera |
| --- | --- | --- | --- |
| `drivetrain.py` | The one motor controller: pin map, gpiozero wiring, `drive(left, right)`. Imported, not run | — | No |
| `person_follow.py` | Autonomous follow loop: detect, steer, search when the target is lost | Yes | Yes |
| `rover.py` | Keyboard teleop — WASD, two pivot modes, live speed trim, safe-stop on any unknown key | Yes | No |
| `detect.py` | Camera + YOLO inference logger. Prints per-frame detections and an EMA fps figure; no motion | No | Yes |
| `motor_test.py` | One-shot drivetrain check: forward 2 s → stop → pivot right 2 s → stop | Yes | No |
| `rover_learning.json` | Snapshot of tuned values from earlier sessions. **No script reads or writes it.** | — | — |

`import person_follow` has no side effects: the model, camera, and GPIO pins are
claimed inside `main()`, so the tuning constants are readable (and the module
testable) on a machine with no hardware attached.

## Documentation

| Document | Contents |
| --- | --- |
| [docs/hardware.md](docs/hardware.md) | GPIO pin map, physical header pins, motor channel wiring, camera and power notes |
| [docs/setup.md](docs/setup.md) | Environment on the Pi, installing dependencies, running each script, headless/SSH behaviour |
| [docs/architecture.md](docs/architecture.md) | Control loop, steering maths, lost-target search states, adaptive low-light pipeline |
| [docs/tuning.md](docs/tuning.md) | Every tuning constant, what it does, and the measured values behind it |
| [docs/code-notes.md](docs/code-notes.md) | What the 2026-09-13 review fixed, what is still risky, and gaps worth knowing before extending |

## Hardware in one table

Two GPIOzero `Motor` objects, one per side, each driving the two wheels on that
side through a dual H-bridge driver:

| Side | `Motor(...)` kwargs | BCM GPIO | Header pin |
| --- | --- | --- | --- |
| Left (motors 1 & 2) | `forward=23, backward=24, enable=12` | 23 / 24 / 12 | 16 / 18 / 32 |
| Right (motors 3 & 4) | `forward=27, backward=22, enable=13` | 27 / 22 / 13 | 13 / 15 / 33 |

Those six numbers live in `drivetrain.py` and nowhere else; the teleop, the follow
loop, and the bench test all read them from there. Speeds are handled as 0–100
integers in the code and divided by 100 before reaching gpiozero, whose
`Motor.forward(speed)` / `Motor.backward(speed)` accept a 0.0–1.0 fraction. The
enable pins are the two hardware-PWM-capable GPIOs (12 and 13).

## Quick start

Run from the project directory on the Pi:

```bash
./yolo_env/bin/python motor_test.py       # 1. drivetrain sanity check
./yolo_env/bin/python detect.py           # 2. camera + inference sanity check
./yolo_env/bin/python rover.py            # 3. drive it by hand
./yolo_env/bin/python person_follow.py    # 4. autonomous follow
```

Rebuilding the environment from scratch, or on a second board:

```bash
python3 -m venv yolo_env
./yolo_env/bin/pip install -r requirements.txt
```

`rover.py` needs a real terminal and exits with an error message if stdin is a
pipe. `person_follow.py` and `motor_test.py` detect the absence of gpiozero: the
follow loop prints `*** SIMULATION MODE ***` and logs its motor decisions, and
`motor_test.py` exits 1, since a drivetrain test without a drivetrain proves
nothing.

Stop any running script with `Ctrl+C`. All four scripts stop the motors on the way
out: `person_follow.py` installs explicit `SIGINT`/`SIGTERM` handlers (plus a
watchdog thread that stops the drivetrain if the control loop stops issuing
commands), `rover.py` and `motor_test.py` do it in a `finally`, and
`detect.py` has no drivetrain.

## What the follow loop actually does

`person_follow.py` captures 640×480 frames, finds the largest box whose class name
is `person`, and derives a lateral offset from the frame centre. A proportional
term on that offset speeds one side and slows the other, so the robot steers
toward the person while continuing to move forward. If the box gets taller than
`TOO_CLOSE_HEIGHT_RATIO` of the frame height, the robot stops rather than running
into you.

When no person is in frame, the robot does not freeze: it keeps scanning in the
direction the person was last drifting, then — after a few seconds — alternates
the scan direction until it either reacquires the target or has never seen one, in
which case it stops. The full state logic, with numbers, is in
[docs/architecture.md](docs/architecture.md).

Current tuned defaults (as committed): `FORWARD_SPEED = 80`, `CONFIDENCE_THRESHOLD
= 0.45`, `INPUT_SIZE = 416`, `MIN_TRACK_SPEED = 35`, `TOO_CLOSE_HEIGHT_RATIO =
0.99`, `MOTOR_WATCHDOG_S = 0.75`, `FRAME_SKIP = 0`. Every constant is in the
`CONFIG` block at the top of the file — see [docs/tuning.md](docs/tuning.md).

## Current state of the hardware

Verified over SSH on the Pi used for development (hostname `otwos`, Raspberry Pi 5
Model B Rev 1.1, Debian 13 trixie, kernel `6.18.29+rpt-rpi-2712`), including a bench
run on 2026-09-13 with the camera and L298N wired:

- **Camera works.** A Logitech HD Pro Webcam C920 on USB (`uvcvideo`) presents
  `/dev/video0`, holds 640×480 at the requested rate, and delivered 60/60 frames at
  ~26 fps. `rpicam-hello --list-cameras` still says "No cameras available" because
  that tool only lists ribbon-connector cameras seen by libcamera; the scripts go
  through OpenCV/V4L2, which sees the webcam fine.
- **Detection works.** Over 25 s of live video, YOLOv8n at `imgsz=416` found a person
  in 152 of 181 inference frames at confidence 0.53–0.91, running 7.4 inferences/s
  end to end.
- **The drivetrain path is clean.** `motor_test.py` completed its forward → stop →
  pivot sequence with no gpiozero errors, so GPIO 12/13/22/23/24/27 were claimed and
  hardware PWM on 12/13 was accepted. `person_follow.py` produced sensible steering
  for 25 s — `following left | left=44% right=100%` narrowing to `left=73% right=87%`
  as the lateral offset fell from −194 px to −39 px. Whether each wheel physically
  turned is a bench observation, not something the software can prove.
- Motor pins are defined in software only. Nothing in the repo records the chassis,
  motor model, driver module part number, or battery — see the unknowns list in
  [docs/hardware.md](docs/hardware.md).

Measured on that board, YOLOv8n on CPU runs a 416-pixel inference in ~0.06 s (≈17/s)
in isolation and ~7.4/s inside the real loop once capture and pre-processing are
included; 640-pixel inference takes ~0.11 s (≈9/s). Details and caveats in
[docs/code-notes.md](docs/code-notes.md#6-performance).

## Repository conventions

- `yolo_env/` (5.1 GB on the Pi) and `rover_data/` (capture folders from
  `detect.py`) are gitignored. Never commit the virtualenv; it is aarch64-specific
  and rebuildable from `requirements.txt`.
- `yolov8n.pt` is **not** tracked. Ultralytics' own weights are AGPL-licensed and
  ~6 MB; the library downloads the file into the working directory on first run, so
  a fresh clone needs one network-connected run (or a manual copy) before
  `person_follow.py` or `detect.py` will start. `MODEL_PATH` is resolved relative to
  the current directory, so run from the project root.
- `rover_learning.json` is tracked deliberately: it is small, hand-written, and the
  only record of earlier tuning sessions.
- `LICENSE` is AGPL-3.0, chosen because `ultralytics` is AGPL-3.0 and this program
  links against it. See
  [docs/code-notes.md](docs/code-notes.md#4-licensing-and-repository-hygiene)
  for the way out if that license is ever a problem.
- `origin` points at `https://github.com/dsribalaji/rover.git`.
