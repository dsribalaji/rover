# Hardware

Everything here is read out of the source files or measured on the board. The
parts the code cannot tell us — chassis, motor model, driver module part number,
battery — are listed at the bottom as unknowns rather than guessed.

## Development board

Measured over SSH on the machine the project runs on (separate from the laptop
that drives it):

| Property | Value |
| --- | --- |
| Model | Raspberry Pi 5 Model B Rev 1.1 (`/sys/firmware/devicetree/base/model`) |
| Hostname | `otwos` |
| OS | Debian GNU/Linux 13 (trixie), Raspberry Pi OS kernel `6.18.29+rpt-rpi-2712` (BCM2712) |
| CPU / memory | 4 cores, ~15 GiB usable RAM (16 GB board) |
| Python | 3.13.5 (`/usr/bin/python3.13`) |
| Login user | `su` — in groups `gpio`, `video`, `input`, `i2c`, `spi`, `dialout`, `sudo`, `docker` |

The `gpio` and `video` group memberships are what let the scripts open
`/dev/gpiochip*` and `/dev/video*` without root. GPIO character devices present:
`/dev/gpiochip0` (alias `gpiochip4`), `gpiochip10`–`gpiochip13`.

## Drivetrain

Four motors, wired as two independent channels — one per side — so each side
steers by varying its own two wheels together. This is a skid-steer layout, and
the code leans on it for the pivot modes.

Channel wiring, defined once in `drivetrain.py` as `LEFT_FORWARD` / `LEFT_BACKWARD` /
`LEFT_ENABLE` and `RIGHT_FORWARD` / `RIGHT_BACKWARD` / `RIGHT_ENABLE`, which the other
scripts import — so a rewiring is a one-file change:

| Channel | gpiozero `Motor` | Direction pins (BCM) | Enable / PWM pin (BCM) |
| --- | --- | --- | --- |
| Left — motors 1 & 2 | `Motor(forward=23, backward=24, enable=12)` | 23, 24 | 12 |
| Right — motors 3 & 4 | `Motor(forward=27, backward=22, enable=13)` | 27, 22 | 13 |

Mapped onto the 40-pin header, since gpiozero numbers by BCM:

| Signal | BCM GPIO | Header pin | Function |
| --- | --- | --- | --- |
| Left IN1 | 23 | 16 | direction |
| Left IN2 | 24 | 18 | direction |
| Left ENA | 12 | 32 | PWM speed |
| Right IN3 | 27 | 13 | direction |
| Right IN4 | 22 | 15 | direction |
| Right ENB | 13 | 33 | PWM speed |
| GND | — | 6 (and 9, 14, 20, 25, 30, 34, 39) | common ground with the driver |

GPIO 12 and GPIO 13 are the Pi's two hardware PWM outputs (PWM0 and PWM1) on the
header, which is what the `enable` line of a gpiozero `Motor` wants for speed
control.

### Driver module

The code speaks to the driver through the label set `ENA / IN1 / IN2 / IN3 /
IN4 / ENB` — the dual-channel pin naming used by L298N-style dual H-bridge
breakouts. No part number is recorded anywhere in the project, so treat the model
as "dual H-bridge, L298N-compatible pinout" and confirm on the bench before
re-wiring. That bench check is `motor_test.py`: it drives bare `Motor` objects on the
shared constants, so it confirms the pins themselves are reachable, and exits 0 only if
the whole sequence ran; any error, Ctrl+C, or missing gpiozero exits 1. Two
consequences worth remembering:

- Both direction lines and the enable line must come from the Pi's 3.3 V logic, so
  a 5 V-supplied L298N needs its logic high threshold met at 3.3 V (true for the
  common modules, but the reason a jumper or level shift sometimes appears on the
  EN pins).
- Speed is the duty cycle on the EN lines. Nothing in the code limits current or
  ramps acceleration, so a hard direction change — a teleop pivot, or a search scan
  that flips a wheel from forward to backward — is a full stall-and-reverse step for
  that side.

### Speed conventions

`TwoChannelMotorController` in `drivetrain.py` takes integer percentages, divides the
magnitude by 100 for gpiozero, and uses the sign to choose the direction pins:

| Value in code | gpiozero sees | Effect |
| --- | --- | --- |
| `60` (`self.speed`, the class default) | `0.60` | 60 % duty on the EN line |
| `MAX_SPEED` = `100` (`increase_speed` ceiling) | `1.00` | full |
| `MIN_SPEED` = `20` (`decrease_speed` floor) | `0.20` | crawl |
| `SPEED_STEP` = `10` | one trim step | what `+` / `-` move by in teleop |
| `-(speed // 2)` inside `turn_left` / `turn_right` | half the magnitude, backward | inside wheel backs up |

Reverse is therefore first-class, not a teleop-only trick: `drive(-40, 40)` pivots left,
`move_backward` takes both channels negative, and following mode reverses the inner wheel
on every search scan (see [architecture.md](architecture.md)).

## Teleop keyboard

`rover.py` reads raw keystrokes through `tty.setraw` on `stdin`, so it must run on
a real terminal — an SSH session is fine, a piped or redirected stdin is not, and
there the loop stops the motors and exits rather than spinning on the closed read.

| Key | Action | Key | Action |
| --- | --- | --- | --- |
| `W` / `w` | forward | `Space` | stop (emergency) |
| `S` / `s` | backward | `+` / `=` | speed +10 % |
| `A` / `a` | gentle left turn | `-` / `_` | speed −10 % |
| `D` / `d` | gentle right turn | `Q` / `q` | pivot left (sharp) |
| `X` / `x` or `Ctrl+C` | stop and exit | `E` / `e` | pivot right (sharp) |
| any other key | stop, and report an unknown key | | |

The fall-through-to-stop behaviour is deliberate: an unmapped key is treated as
"do nothing dangerous". Raw mode clears ISIG, so `Ctrl+C` reaches the loop as the
`0x03` byte and exits through the same path as `X`.

## Camera

The scripts ask OpenCV for a V4L2 capture node, not for a libcamera stack:

- `person_follow.py`: `cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_V4L2)` at
  `CAMERA_WIDTH`×`CAMERA_HEIGHT` (640×480), 30 fps requested, buffer size 1, FourCC
  forced to `YUYV`, with a fallback to `cv2.VideoCapture(0, cv2.CAP_V4L2)` if the named
  node will not open. The follow loop sizes its maths from `frame.shape`, so an
  unexpected resolution still steers relative to the image in hand.
- `detect.py`: `cv2.VideoCapture(device, cv2.CAP_V4L2)`; `--device` takes an index or a
  `/dev/video` path and defaults to `/dev/video0`, frames are resized to
  `--width`×`--height` (640×480 default), and it warns if the node reports a size other
  than the request.

Measured state on the Pi, from a bench run on 2026-09-13: `/dev/video0` is a
**Logitech HD Pro Webcam C920** on USB, driver `uvcvideo`. It accepts the 640×480
request at 30 fps and delivered 60/60 frames at ~26 fps in a straight read loop. Mean
frame brightness was 135.9 in a lit room, which sits comfortably between the
`LOW_LIGHT_THRESHOLD` (72) and `BRIGHT_LIGHT_THRESHOLD` (145) gates, so neither the
CLAHE path nor the auto-gain loop engages in those conditions.

Note that `rpicam-hello --list-cameras` still prints `No cameras available!` with the
webcam connected — that tool only reports ribbon-connector cameras through libcamera.
The board's own ISP and codec nodes (`pispbe` on `/dev/video20`–`/dev/video35`,
`rpi-hevc-dec` on `/dev/video19`) are unrelated to what these scripts open, and
`picamera2 0.3.36` is installed but imported by nothing. `/boot/firmware/config.txt`
does carry `camera_auto_detect=1`, so a Pi camera module would also work on the same
V4L2 path.

Practical implication: either camera type lands on `/dev/video0` and the scripts run
unchanged. The USB webcam is the configuration that has actually been tested.

## Power and safety notes

- There is no recorded wiring for the battery, no low-voltage monitoring, and no
  hardware watchdog. `person_follow.py` does have a software one: `motor_watchdog()`
  stops the drivetrain if the control loop goes `MOTOR_WATCHDOG_S` (0.75 s) without
  commanding it, and it is a thread inside that process, so it cannot help a brown-out.
  Teleop has no timeout at all — an untouched key keeps the last command running. If the
  Pi and the motor rail share a supply, a brown-out mid-pivot can leave the driver
  latched with the last PWM value applied.
- No obstacle, cliff, or distance sensing exists in the code. The only thing that
  stops the robot for its own reasons is a person box taller than
  `TOO_CLOSE_HEIGHT_RATIO` of the frame, plus the loss-of-target stop.
- `person_follow.py` installs `SIGINT` and `SIGTERM` handlers that stop both
  motors and release the camera; `rover.py` does the same through its
  `try/except/finally`. Killing either with `SIGKILL` skips the cleanup and can
  leave the drivetrain powered.

## Unknowns, to fill in on the bench

Not derivable from this repository, and deliberately not guessed here:

- Motor part number, voltage, stall current, and gear ratio
- Exact driver module and whether its EN jumpers are fitted
- Chassis, wheel diameter, track width, battery chemistry and capacity
- Camera mounting height and pitch — the sensor is a Logitech C920, but its
  angle changes the pixel-to-distance assumption behind `TOO_CLOSE_HEIGHT_RATIO`
- Whether the motor and logic supplies are common-grounded on purpose
