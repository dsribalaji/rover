# Setup and running

Target machine is the Raspberry Pi 5 described in [hardware.md](hardware.md). The
laptop that hosts this repository and the `gh` CLI is a separate box; nothing in
the project builds or runs there.

## The existing environment

The project runs from a virtualenv at `yolo_env/` inside the repository. Measured
properties:

| Property | Value |
| --- | --- |
| Created by | `/usr/bin/python3 -m venv /home/su/rover/yolo_env` (recorded in `yolo_env/pyvenv.cfg`) |
| Base interpreter | Python 3.13.5, `/usr/bin/python3.13` |
| `include-system-site-packages` | `true` — distro Python packages are visible inside the venv |
| Size on disk | 5.1 GB |
| Git status | ignored by `.gitignore` (`yolo_env/`), deliberately never committed |

Because system site-packages are on, `pip list` inside the venv also shows base-image
packages (`cloud-init`, `Flask`, a distro `opencv 4.10.0`, and so on). Those are not
project dependencies. `cv2` itself resolves to the venv's `opencv-python` 4.13.0, and
`ultralytics` to the venv's 8.4.48.

Direct dependencies, as installed when this documentation was written:

| Package | Version | Used by |
| --- | --- | --- |
| `ultralytics` | 8.4.48 | `person_follow.py`, `detect.py` |
| `opencv-python` | 4.13.0.92 | all camera work |
| `numpy` | 2.4.4 | `detect.py` typing, OpenCV buffers |
| `gpiozero` | 2.0.1 | all motion, through the shared `drivetrain.py` (`lgpio 0.2.2.0` / `gpiod 2.2.0` present as backends) |
| `torch` | 2.11.0+cu130 | YOLO runtime; reports CUDA **unavailable** on the Pi, so inference is CPU |
| `torchvision` | 0.26.0 | YOLO runtime |
| `pillow` | 12.2.0 | YOLO runtime |
| `matplotlib`, `scipy` | 3.10.9, 1.17.1 | YOLO runtime |
| `picamera2` | 0.3.36 | installed, **not imported by any script** |

`requirements.txt` pins only what the project imports directly. The repository root
carries the GNU AGPL-3.0 text as `LICENSE`, because `ultralytics` is AGPL-3.0 and the
rover code links against it.

## Fresh install

```bash
sudo apt update && sudo apt install -y python3-venv python3-numpy
python3 -m venv yolo_env
./yolo_env/bin/pip install --upgrade pip
./yolo_env/bin/pip install -r requirements.txt
```

The board's venv carries torch `2.11.0+cu130` — a CUDA-tagged build — together with
`cuda-toolkit 13.0.2` and `cuda-bindings 13.2.0` wheels. The Pi has no NVIDIA GPU and
`torch.cuda.is_available()` reports `False`, so that suffix buys nothing here and a CPU
build is enough (and much smaller). `requirements.txt` pins plain `torch==2.11.0`, which
resolves to whatever aarch64 wheel PyPI publishes for that version; if that ever fails to
install on a Pi, drop the line and let `ultralytics` pull a compatible build.

## Model weights

`person_follow.py` uses `MODEL_PATH = 'yolov8n.pt'` and `detect.py` defaults to
`--model yolov8n.pt`. Both are relative paths, so **run from the repository root** —
the name is resolved against the current directory, not against the file's location.

The weights file (6.2 MB) is no longer tracked: `.gitignore` now ignores `*.pt`,
`*.pth` and `*.ckpt`. The copy on the Pi's disk is untouched by that change and keeps
the board running; it is simply no longer part of the repository, so a fresh clone has
no weights file in it. Ultralytics treats a bare, recognised model name as something to
fetch when the file is absent, so the first run after a clone downloads it rather than
failing. Nothing else in the repo needs a download step. `detect.py` also accepts
`--model` for any other Ultralytics checkpoint.

## Preflight checks

```bash
# camera present and readable?
ls -l /dev/video0
rpicam-hello --list-cameras          # Pi camera module; "No cameras available!" = none attached

# GPIO reachable without sudo? (user must be in the gpio group)
ls -l /dev/gpiochip0
groups | tr ' ' '\n' | grep -E 'gpio|video'
```

On the development Pi this currently resolves to a Logitech HD Pro Webcam C920 on
USB, and both camera scripts run against it. `rpicam-hello` reporting "no cameras"
does not mean the scripts will fail — that tool only sees ribbon-connector cameras,
not the V4L2 node OpenCV uses.

## Running

```bash
cd ~/rover
./yolo_env/bin/python motor_test.py
```

Runs a fixed sequence with printed step numbers: both channels forward at 100 % for
2 s, stop for 1 s, pivot right (left forward / right backward at 100 %) for 2 s,
stop. Expect motion, then silence. The pins come from `drivetrain.py`, which the script
imports as bare constants rather than through the controller class, so the test also
confirms the pins themselves are reachable. `Ctrl+C` or any exception prints a line and
the `finally` still stops and closes both channels; the process exits 1 unless the whole
sequence ran, and it exits 1 immediately with `ERROR: gpiozero is not importable` when
the library is missing. This is the fastest way to find a swapped direction pair or a
dead channel.

```bash
./yolo_env/bin/python rover.py
```

Checks that stdin is a terminal first and refuses to start with
`ERROR: rover.py needs a terminal for raw key input.` under a pipe, `cron` or redirected
stdin, so it works in an interactive SSH session only. Then it prints a control panel and
waits for single keystrokes — no Enter. Keys:

`W`/`S` forward/back, `A`/`D` gentle turn, `Q`/`E` sharp pivot, `Space` stop,
`+`/`-` speed ±10 % (`SPEED_STEP`, inside the 20–100 window set by `MIN_SPEED` and
`MAX_SPEED`), `X` exit. Every other key stops the robot and labels itself unknown.
Status prints in place on one line: `[Speed:  60%] ⬆️  Moving FORWARD`. Raw mode clears
`ISIG`, so `Ctrl+C` arrives as the `0x03` byte and takes the same exit path as `X`. If
stdin closes mid-session (dropped SSH, exhausted pipe) the loop stops the motors and
exits rather than spinning on a dead descriptor. Cleanup (stop, then close both GPIO
channels) runs on every exit path and on any exception.

```bash
./yolo_env/bin/python detect.py --device /dev/video0 --conf 0.3
```

Prints one line per inference:

```
<ISO timestamp> | frame=<n> | size=<W>x<H> | fps=<EMA> | detections=<class>:<count>(@<conf>), … | elapsed=<s>s
```

Useful flags: `--model` (weights), `--conf` (default 0.25), `--imgsz` (default 416,
which now matches the follower), `--device` (default `/dev/video0`; a bare index such as
`0` works too, and the camera is opened with `cv2.CAP_V4L2` either way),
`--width`/`--height` (640/480, also the resize target applied before inference). Exit
codes: `2` camera would not open, `3` `ultralytics` not importable, `4` model failed
to load. `--interval` and `--skip` no longer exist in the parser, so passing either is an
unknown-argument error now rather than a silent no-op. The script prints a warning with
the resolution the camera actually reports when that differs from what was asked for.

```bash
./yolo_env/bin/python person_follow.py
```

The main event. Startup prints, in order: `Loading YOLOv8n model...`,
`Model loaded | Input Size: 416px`, `Camera opened!`, then
`Starting person-follow mode (Input Size=416, Conf=0.45)` and `Press Ctrl+C to stop.`
If gpiozero is not importable a `*** SIMULATION MODE ... ***` banner goes up before the
model load, and those printed actions are the only thing that happens.

Runtime output, in the shapes the code prints (numbers below are illustrative, not
measurements):

- `Person: Conf=0.62, Offset=48, Height=271` — one line per inference that found a person
- `>>> ROBOT ACTION: following right | left=92% right=68%` — only when the action changes
- `Camera tune | Light=61 Conf=0.41 Gain=12.0 Exp=-8.0` — when the auto-exposure loop changes a setting
- `FPS: 6.3 | Light=118 | ConfEMA=0.44` — every 20th inference
- `WARNING: no frame from /dev/video0 (20 in a row)` — camera dropped a read; the first failure prints, then every 20th, and the drivetrain is held stopped meanwhile
- `>>> WATCHDOG: drivetrain stopped, no control-loop command for 0.75s` — once per stall

There are no command-line arguments; everything is configured by editing the
`# ===== CONFIG =====` block at the top. See [tuning.md](tuning.md). `Ctrl+C` (or
`SIGTERM`) prints `Stopping...`, stops both motors, closes the GPIO channels, releases
the camera, and exits 0; a camera that will not open prints
`Camera could not be opened.` and exits 1. A daemon `motor_watchdog` thread calls
`stop_all()` whenever the control loop goes longer than `MOTOR_WATCHDOG_S` without
commanding the drivetrain, so a wedged process cannot leave the rover driving away; it
does not cover `SIGKILL` or a power cut.
Nothing is loaded, opened or claimed at import time any more — all of it happens inside
`main()`, behind `if __name__ == "__main__"` — so the CONFIG block and the steering maths
can be imported on a laptop with no camera and no GPIO.

## Running headless

No script in the repo opens a window (`cv2.imshow` appears nowhere), so nothing needs a
display and all of them run over plain SSH. `person_follow.py` used to set
`QT_QPA_PLATFORM=offscreen` itself; that line is deleted, since it ran after `import cv2`
and therefore never influenced OpenCV's GUI backend selection in the first place. If you
want the variable set anyway, export it before launching:

```bash
QT_QPA_PLATFORM=offscreen ./yolo_env/bin/python person_follow.py
```

## Log capture

The scripts print to stdout only. To keep a record of a tuning session:

```bash
./yolo_env/bin/python person_follow.py 2>&1 | tee follow-$(date +%F-%H%M).log
```

Log files are covered by the `*.log` rule in `.gitignore`, so they do not pollute
the repository. Note the historical `vision-mqtt.log` (8.4 MB) and
`drone-mqtt.log` (206 KB) sitting in the Pi's home directory belong to other
projects, not to this one.
