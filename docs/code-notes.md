# Code notes

Verified observations about the source, plus the gaps that would slow down the next
contributor. Nothing here is a style opinion; each item was checked against the files,
the installed libraries, or the board itself.

Legend: **dead** = present but has no effect; **risk** = will bite under specific
conditions; **gap** = missing capability the code implies but does not do.

A code-review pass on 2026-09-13 fixed most of what this file used to list as open.
Section 1 records what changed and where it went, so the next reader does not
re-discover it. Sections 2 to 6 are what is still true today.

## 1. Fixed on 2026-09-13, in one place

Referenced by symbol rather than line number, because line numbers rot — the earlier
version of this file cited fourteen of them and all fourteen went stale within one edit.

| Was | Now |
| --- | --- |
| `CENTER_THRESHOLD`, `STEER_SPEED`, `SEARCH_TURN_SPEED` defined but never read; `detect.py --interval` and `--skip` parsed but never read; `last_log_time`/`fps_acc_*` initialised but never used | deleted |
| `SKIP_FRAMES = 1` with `frame_count % SKIP_FRAMES`, which made the knob inert and `0` a `ZeroDivisionError` | `FRAME_SKIP` in `person_follow.py`: `0` infers every frame, `N` infers the first frame then every `(N+1)`th |
| Two search-ladder branches (within `SEARCH_HOLD_TIME` and within `SEARCH_ALT_START_TIME`) that issued identical commands and differed only in the printed string | one branch; `SEARCH_HOLD_TIME` now only selects the `hold`/`extended` label, which is what it already did |
| `drive()` mapped any non-positive side to `stop`, so following could never reverse a wheel | `drivetrain.TwoChannelMotorController.drive()` takes signed percentages: positive forward, negative reverse, zero stop |
| `TOO_CLOSE_HEIGHT = 600` — unreachable on a 480-row frame, so the proximity stop could never fire | `TOO_CLOSE_HEIGHT_RATIO` compared against the real frame height; see section 3 for why the default is still near 1.0 |
| `person_follow.py` loaded the model, opened the camera and claimed GPIO at import, so nothing was testable without hardware | setup and loop live in `main()` behind `if __name__ == "__main__":` |
| `rover.py` and `person_follow.py` each defined their own `TwoChannelMotorController`, with different capabilities, and the pin numbers were written down in three files | one `drivetrain.py` holds the pin constants and the class; all three scripts import it |
| `shutdown()` called `stop_all()` then `cleanup()`, then `sys.exit(0)` from inside a `finally`, so the loop's own `finally` ran it a second time against closed pins and any in-flight error exited 0 | `shutdown()` is idempotent, stops the watchdog, cleans up once, releases the camera, and never raises `SystemExit`; SIGINT/SIGTERM set `running = False` |
| A dropped `cap.read()` hit `continue` with no pause, spinning one core while the drivetrain held its last command | warns once, calls `stop_all()`, sleeps 50 ms, keeps beating the watchdog |
| Nothing timed a stalled control loop | `motor_watchdog` daemon thread stops the drivetrain after `MOTOR_WATCHDOG_S` without a command, and says so on the console |
| `os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")` ran after `import cv2`, so it could not affect backend choice | removed; nothing in the repo calls `imshow` |
| `rover.py` advertised Ctrl+C, but raw mode clears `ISIG`, so Ctrl+C arrived as the byte `0x03` and hit the "unknown key" branch: the robot stopped, the program did not exit | `'\x03'` is handled as the exit key, and closed stdin now exits instead of spinning |
| `motor_test.py` ended in `finally: sys.exit()`, so a wiring failure exited 0 | exits 1 on any failure or interrupt, 0 only on a complete run |
| No `LICENSE`, and `yolov8n.pt` (6.2 MB) committed | `LICENSE` added (AGPL-3.0, matching `ultralytics`); weights untracked and gitignored, ultralytics fetches them on first run |
| `.gitignore` ignored `*.json` wholesale with one negation | that rule replaced by `rover_data/`, so new JSON is tracked by default |
| `FRAME_SKIP` used `frame_count % (FRAME_SKIP + 1)`, which threw away the very first frame | `(frame_count - 1) % (FRAME_SKIP + 1)`, so frame 1 is always inferred on |
| A skipped frame reached `continue` before the loop's `_beat()`, so raising `FRAME_SKIP` could trip the watchdog while the loop was perfectly healthy | skipped frames beat the watchdog too; it now means "the loop is not turning", which is what it was meant to detect |
| `motor_watchdog` did `except Exception: return`, so one failed `stop_all()` killed the safety net permanently — in the exact situation that needs it | logs the failure once per stall episode and keeps retrying every 100 ms |
| `MOUNT_OFFSET_PX`'s comment ("positive means the camera looks right of centre") contradicted the arithmetic `offset = centre − frame_width/2 − MOUNT_OFFSET_PX`, which needs a negative value for that mount | comment states the calibration instead: paste the mean `Offset=` printed when a person stands dead ahead (−73 px on the 2026-09-13 bench) |
| `rover.py` ran `controller.run()` and dropped its result, so the "needs a terminal" guard printed an error and still exited 0 | `sys.exit(controller.run())` |
| `detect.py --device 0` was ambiguous: pinning `cv2.CAP_V4L2` means a bare index resolves to `/dev/videoN`, not to OpenCV's probe order | behaviour kept (it matches the follower), stated in `--help` |
| `open_camera()` configured resolution, fps, buffer and FOURCC *before* checking `isOpened()`, so the index-0 fallback ran unconfigured — and the failed first handle was never released, leaving `/dev/video0` claimed | the fallback releases the first handle and the settings are applied after a successful open, so both paths are configured identically |
| `cleanup()` left the controller looking usable, so a control loop still mid-iteration when a signal handler cleaned up hit closed pins and died with `GPIODeviceClosed` (seen on the bench on 2026-09-13) | `cleanup()` clears `available` last, making every motor method inert after close |

## 2. Still worth knowing before you drive it

**The camera-read failure path now stops, but it does not recover.** On a dropped
frame the loop warns, calls `stop_all()`, sleeps 50 ms and retries. A camera that
re-enumerates after a USB glitch (`/dev/video0` reassigned, or the C920 reappearing
under a different index) leaves the rover stopped with the device handle open on the
old node. Restarting the script is the current remedy. **gap**

**The motors are only safe by the watchdog, not by the driver.** There is no
`max_time=` limit on the gpiozero pins, so a stall longer than `MOTOR_WATCHDOG_S` is
caught only by a Python thread in the same process. A kernel-level hang, or a process
killed with `SIGKILL`, leaves the GPIO state as it was; gpiozero's own atexit cleanup
does not run for `SIGKILL`. **risk**

**Two rover scripts cannot run at once, and a fast restart can look like two.** Claiming
a pin another process holds surfaces as `lgpio.error: 'GPIO busy'`; `TwoChannelMotorController`
now converts that into a named `RuntimeError` listing the six BCM numbers and exits 1,
but the underlying release is not instantaneous: restarting `person_follow.py` within a
second of the previous run exiting hit this on the bench on 2026-09-13, and the same
command re-run a moment later started normally. Wait a second. **risk**

**The target can change identity between frames.** Selection is "largest box whose
class name is `person`", with no track IDs and no persistence (`model.track()` and
`results[0].boxes.id` are unused). Two people in frame, or one partly occluded, lets
the "largest" box jump — the robot then steers at a different target with no
hysteresis. `detect_conf_ema` smooths confidence for the auto-exposure gate only, not
for target choice. **gap**

**`rover.py` teleop has no dead-man timeout.** It is edge-triggered: a key sets the
state and that state persists until the next key. Closing the terminal now stops the
robot, and any unmapped key stops it, but an operator who simply stops typing leaves it
driving. A hold-to-drive timeout is the missing piece. **risk**

**Simulation mode prints real-looking output.** With gpiozero missing, `available` is
`False`, every motor method returns early, and the `ROBOT ACTION` lines still print.
`person_follow.py` prefixes startup with a `SIMULATION MODE` banner and
`motor_test.py` refuses to run at all, but it is still easy to mistake those log lines
for motion in an old transcript. **risk**

**The speed floor makes steering a ratio, not an authority range.** `MIN_TRACK_SPEED = 35`
against `FORWARD_SPEED = 80` means the inner wheel never drops below about 44 % of the
outer one while following. Turning authority comes from that ratio, not from reversing
a wheel. Reversing is now reachable in code — `turn_left`, `turn_right`, `pivot_left`
and `pivot_right` all command it — and is still unmeasured on the bench. **design**

## 3. Hardware limits the code cannot fix

**The camera mount saturates the vertical field.** At the current low, upward-tilted
position a person fills the frame from roughly a metre out: the 90-second bench run
measured box heights with a median of 459 of 480 rows and a maximum of 478.
`TOO_CLOSE_HEIGHT_RATIO` is therefore `0.99`, a threshold that fires only when someone
is nearly against the rover. Remount the camera higher and tilted down, then tighten it
towards 0.85 and let it act as a real standoff.

**The measured steering bias is mount yaw, not code.** The same run put the person at a
mean offset of −73 px while standing near the centreline, i.e. the camera is not
pointed straight along the chassis. `MOUNT_OFFSET_PX` exists so this is a one-line
calibration instead of a re-drill: stand on the centreline, run for ten seconds, and
set whatever mean offset remains.

**No obstacle awareness of any kind.** The robot will drive into a wall while a person
stays centred in frame. **gap**

## 4. Licensing and repository hygiene

- **`LICENSE` is GNU AGPL-3.0**, chosen because the project depends on `ultralytics`,
  whose installed metadata reports `License: AGPL-3.0` with the AGPL text as its
  license file (`opencv-python` Apache 2.0, `gpiozero` BSD-3-Clause, `torch`
  BSD-3-Clause, all checked from the venv metadata). If you would rather not carry
  AGPL, the way out is swapping the detector for a permissively-licensed model, not
  editing the license file. A robot that later exposes a web control page engages
  AGPL's network clause. **risk**
- **Weights are no longer in git**, so a fresh clone needs one run to fetch them.
  `MODEL_PATH = 'yolov8n.pt'` resolves relative to the working directory: still run
  from the repository root.
- **`yolo_env/` is 5.1 GB and correctly ignored.** Never force-add it; the wheel is
  aarch64-specific and the tree includes compiled CUDA bindings.
- Commit identity: the repository sets a **repo-local** `user.name` / `user.email`,
  which overrides the global config. The copy on the Pi carries the identity that was
  rsynced across with `.git` (`su <su@local>`, which does not map to a GitHub account),
  so a commit made on the board still lands as an unlinked author unless you set it
  there first.

## 5. Missing capabilities, in rough order of payoff

1. **Telemetry.** Nothing records what actually happened: no per-frame CSV, no event
   log, no way to answer "why did it lose me in that doorway". `rover_learning.json`
   is the fossil of an attempt at this.
2. **A tuned-parameters file that is loaded.** Move the CONFIG block into
   `config.json` or a `--config` YAML, read it at startup, and the numbers in
   `rover_learning.json` become real again instead of a stale side note.
3. **Target persistence** — `model.track()` with IoU/ID continuity, or at minimum
   prefer the previous box's position over the raw largest-area pick.
4. **Camera recovery.** Reopen `cap` after N consecutive read failures instead of
   waiting stopped.
5. **Depth or distance guard for obstacles.** None exists.
6. **Hold-to-drive timeout in the teleop.**
7. **Automated tests.** `person_follow.py` is now importable without hardware, so the
   steering maths, the search ladder and the watchdog are unit-testable and none are
   tested. The bench script used to check the 2026-09-13 fixes lives on the board at
   `/tmp/rover_verify.py` and is not in the repo.

## 6. Performance

Measured on the Pi 5 with a synthetic random-noise 640×480 frame (`yolov8n.pt`,
torch CPU build, no camera attached — this isolates network cost from capture,
resize, and Python overhead):

| `imgsz` | Steady-state inference | Approx. throughput | First call |
| --- | --- | --- | --- |
| 416 (what `person_follow.py` uses) | 0.059–0.060 s | ≈17 /s | 1.10 s warm-up |
| 640 (what `detect.py` used to default to) | 0.110–0.111 s | ≈9 /s | 0.201 s (already warm) |

Two conclusions. First, `INPUT_SIZE = 416` buys about 1.9× the inference rate of 640,
which is the whole reason the follower does not use the old `detect.py` default.
Second, `half=True` was removed from the follower's model call: this CPU-only build
accepted it silently and it did nothing, half precision needing CUDA.

The real loop runs slower than the isolated figure, as expected: a 25-second run of
`person_follow.py` against a live C920 stream reported `FPS: 7.4` end to end, with
detections in 152 of 181 frames. A later 90-second run after the camera was moved
reported 5.0 inferences/s (median 200 ms, p95 250 ms) on a brighter scene where the
auto-exposure loop is pulling exposure down. `detect.py`'s EMA measures inference time
only, not the end-to-end loop rate, so the two scripts are not directly comparable.

`MOTOR_WATCHDOG_S = 0.75` is set from those numbers: roughly three times the p95
inference time, so normal jitter never trips it and a genuine stall does.
