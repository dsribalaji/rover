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

Channel wiring, identical in `rover.py`, `motor_test.py`, and `person_follow.py`:

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
re-wiring. Two consequences worth remembering:

- Both direction lines and the enable line must come from the Pi's 3.3 V logic, so
  a 5 V-supplied L298N needs its logic high threshold met at 3.3 V (true for the
  common modules, but the reason a jumper or level shift sometimes appears on the
  EN pins).
- Speed is entirely software PWM on the EN lines. Nothing in the code limits
  current or ramps acceleration, so a hard direction change (the pivot calls in
  `rover.py`) is a full stall-and-reverse step for that side.

### Speed conventions

`TwoChannelMotorController` stores speed as an integer percent and divides by 100
before calling gpiozero:

| Value in code | gpiozero sees | Effect |
| --- | --- | --- |
| `60` (default in `rover.py` / `person_follow.py`) | `0.60` | 60 % duty on the EN line |
| `100` (`increase_speed` ceiling) | `1.00` | full |
| `20` (`decrease_speed` floor in `rover.py`) | `0.20` | crawl |
| `speed // 2` inside `turn_left` / `turn_right` | half of the current value | inside wheel backs up |

The stray comment `# Default speed (0.0 to 1.0 in gpiozero)` above `self.speed =
60` describes the gpiozero unit, not the one the class uses. See
[code-notes.md](code-notes.md).

## Teleop keyboard

`rover.py` reads raw keystrokes through `tty.setraw` on `stdin`, so it must run on
a real terminal — an SSH session is fine, a piped or redirected stdin is not.

| Key | Action | Key | Action |
| --- | --- | --- | --- |
| `W` / `w` | forward | `Space` | stop (emergency) |
| `S` / `s` | backward | `+` / `=` | speed +10 % |
| `A` / `a` | gentle left turn | `-` / `_` | speed −10 % |
| `D` / `d` | gentle right turn | `Q` / `q` | pivot left (sharp) |
| `X` / `x` | stop and exit | `E` / `e` | pivot right (sharp) |
| any other key | stop, and report an unknown key | | |

The fall-through-to-stop behaviour is deliberate: an unmapped key is treated as
"do nothing dangerous".

## Camera

The scripts ask OpenCV for a V4L2 capture node, not for a libcamera stack:

- `person_follow.py`: `cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)` at
  640×480, 30 fps requested, buffer size 1, FourCC forced to `YUYV`, with a
  fallback to `cv2.VideoCapture(0, cv2.CAP_V4L2)` if the named node will not open.
- `detect.py`: `cv2.VideoCapture(device)` where `--device` defaults to `0`,
  resized to `--width`×`--height` (640×360 default).

Measured state on the Pi as of the last check: `/dev/video0` does not exist and
`rpicam-hello --list-cameras` prints `No cameras available!`. The only video nodes
are the board's own ISP and codec devices — `pispbe` on `/dev/video20`–`/dev/video35`
plus `/dev/media1` and `/dev/media2`, and `rpi-hevc-dec` on `/dev/video19` plus
`/dev/media0`. `/boot/firmware/config.txt` has `camera_auto_detect=1`, so a camera
attached to the Pi's camera connector would be picked up at boot; `picamera2
0.3.36` is installed in the environment but no script imports it.

Practical implication: connect a camera (Pi camera module on the ribbon connector,
or a USB webcam, which also lands on `/dev/video0`) before running `detect.py` or
`person_follow.py`, or both exit with a camera-open error.

## Power and safety notes

- There is no recorded wiring for the battery, no low-voltage monitoring, and no
  watchdog. If the Pi and the motor rail share a supply, a brown-out mid-pivot can
  leave the driver latched with the last PWM value applied.
- No obstacle, cliff, or distance sensing exists in the code. The only thing that
  stops the robot for its own reasons is a person box taller than
  `TOO_CLOSE_HEIGHT`, plus the loss-of-target stop.
- `person_follow.py` installs `SIGINT` and `SIGTERM` handlers that stop both
  motors and release the camera; `rover.py` does the same through its
  `try/except/finally`. Killing either with `SIGKILL` skips the cleanup and can
  leave the drivetrain powered.

## Unknowns, to fill in on the bench

Not derivable from this repository, and deliberately not guessed here:

- Motor part number, voltage, stall current, and gear ratio
- Exact driver module and whether its EN jumpers are fitted
- Chassis, wheel diameter, track width, battery chemistry and capacity
- Camera model, lens, and mounting height/pitch (changes the pixel-to-angle
  assumption behind `TURN_GAIN` and `TOO_CLOSE_HEIGHT`)
- Whether the motor and logic supplies are common-grounded on purpose
