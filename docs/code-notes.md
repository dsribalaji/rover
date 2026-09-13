# Code notes

Verified observations about the source as committed, plus the gaps that would slow
down the next contributor. Nothing here is a style opinion; each item was checked
against the files, the installed libraries, or the board itself.

Legend: **dead** = present but has no effect; **risk** = will bite under specific
conditions; **gap** = missing capability the code implies but does not do.

## 1. Configuration that does nothing

| Item | Where | Evidence |
| --- | --- | --- |
| `CENTER_THRESHOLD = 70` | `person_follow.py:108` | defined, never referenced again — replaced by `STEER_DEADBAND` + `TURN_GAIN` |
| `STEER_SPEED = 90` | `person_follow.py:110` | defined, never referenced — per-side speeds now derive from `FORWARD_SPEED` |
| `SEARCH_TURN_SPEED = 60` | `person_follow.py:116` | its own comment says "Legacy knob (kept for compatibility)"; `SEARCH_SCAN_SPEED` is the live one |
| `--interval` | `detect.py:34-40` | parsed, never read; `last_log_time` is initialised at line 100 and listed as `nonlocal` at 138 but never compared |
| `--skip` | `detect.py:43` | parsed, never read — `detect.py` has no frame-skipping code at all |
| `rover_learning.json` | repo root | `grep -rn rover_learning *.py` returns nothing; the file is a record of tuning done outside this code |

Two of the six are actively misleading: a reader who edits `STEER_SPEED` expecting
the robot to turn harder sees no change.

## 2. Behaviour quirks worth knowing before you drive it

**`SKIP_FRAMES = 0` would crash.** `person_follow.py:261` computes
`frame_count % SKIP_FRAMES`. The commit sets 1, so every frame runs. Setting 0 to
mean "skip nothing" — the convention `detect.py --skip` uses — raises
`ZeroDivisionError` on the first frame. **risk**

**Two search branches do the same thing.** `person_follow.py:372` (≤ `SEARCH_HOLD_TIME`)
and `:378` (≤ `SEARCH_ALT_START_TIME`) both call `apply_search_scan(last_seen_direction)`
at the same speed and differ only in the printed string, so `SEARCH_HOLD_TIME = 1.5`
currently has no observable effect. The intended design was probably "hold briefly,
then creep"; the implementation scans at one speed until 5 s, then alternates. **dead**

**`Ctrl+C` never prints "Stopped by user.".** `person_follow.py:213` registers
`shutdown` for `SIGINT`, so the `except KeyboardInterrupt` at line 417 cannot fire —
the handler runs first and ends in `sys.exit(0)`. **dead**

**`shutdown()` runs twice on `Ctrl+C`, and the second pass throws internally.** The
handler calls `stop_all()` then `cleanup()` (which closes both channels), then
`sys.exit(0)`. That `SystemExit` unwinds through the loop's `finally: shutdown()` at
line 420, so `robot.stop_all()` is called on closed devices. Verified against the
installed gpiozero 2.0.1 with mock PWM pins: after `close()`, both `stop()` and
`forward()` raise `GPIODeviceClosed` (`close()` again is a no-op). The second
`shutdown()`'s own `finally` then raises a fresh `SystemExit(0)`, which replaces the
error, so the process still exits 0 with no traceback — but if the *first* stop command
had failed, nothing would retry it. **risk**

**Follow mode can never stop or reverse a wheel.** `drive()` maps any non-positive
side to `stop`, and the clamp at lines 350-351 keeps both sides at or above
`MIN_TRACK_SPEED = 35`. So tracking is always both wheels forward; the only autonomous
reverse motion is the inner channel inside `turn_left`/`turn_right` during search.

**The target can change identity between frames.** Selection is "largest box whose
class name is `person`" (lines 301-314), with no track IDs and no persistence
(`model.track()` and `results[0].boxes.id` are unused). Two people in frame, or one
person partly occluded, lets the "largest" box jump — the robot then steers at a
different target with no hysteresis. `detect_conf_ema` smooths confidence for the
auto-exposure gate only, not for target choice. **gap**

**`person_follow.py` is a script, not a module.** YOLO load (line 135), camera open
(line 140), and GPIO claim (line 200) all happen at import time, so you cannot import
its classes for a test or reuse them without touching hardware. `rover.py` and
`detect.py` are guarded properly. **gap**

**The drivetrain class exists twice, with two behaviours.** `rover.py:19` and
`person_follow.py:29` define separate `TwoChannelMotorController` classes:

| Capability | `rover.py` | `person_follow.py` |
| --- | --- | --- |
| `move_backward`, `pivot_left`, `pivot_right` | yes | no |
| `increase_speed` / `decrease_speed` | yes | no |
| `drive(left, right)` for proportional steering | no | yes |
| Works with gpiozero missing | no (hard import) | yes (`GPIO_AVAILABLE` → log-only) |

The GPIO numbers are therefore written down in three files (`rover.py`,
`person_follow.py`, `motor_test.py`); a rewire means three edits and any drift is
silent. **gap**

## 3. Comment vs. reality

- `rover.py:29` — `# Default speed (0.0 to 1.0 in gpiozero)` sits above
  `self.speed = 60`. The class consistently uses 0–100 and divides by 100 at the
  gpiozero boundary; the comment describes the wrong unit.
- `person_follow.py:109` — `TOO_CLOSE_HEIGHT = 600 # High threshold to prevent
  "stopped (too close)" triggers` is honest, and worth reading before assuming a
  proximity stop exists. On a 480-line frame no box can reach 600 px, so it does
  not fire.
- `person_follow.py:130` — `# Keep the script usable over SSH/headless setups` above
  `os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")`, which runs at line 131,
  after `import cv2` at line 14. OpenCV's GUI backend is already chosen by then; the
  line has no effect (harmless, since nothing calls `imshow`). Set the variable in the
  environment instead.
- `detect.py:56` — `# attempt to load YOLO model` sits above the `cap.set(...)`
  resolution calls, two lines before the actual import attempt.

## 4. Licensing and repository hygiene

- **No `LICENSE` file.** The project depends on `ultralytics`, whose installed metadata
  reports `License: AGPL-3.0` with the AGPL-3.0 text as its license file
  (`opencv-python` Apache 2.0, `gpiozero` BSD-3-Clause, `torch` BSD-3-Clause, all
  checked from the venv metadata). The repo currently declares no license of its own,
  which defaults to "all rights reserved" for your code. Decide before this leaves
  your account — a robot that exposes a web control page in future would engage AGPL's
  network clause. **risk**
- **`.gitignore` ignores `*.json`** with a single negation for `rover_learning.json`.
  Any future `params.json`, dataset manifest, or label map is silently untracked. The
  same file also ignores `lib/` and `lib64/` (from the generic Python template), which
  would hide a project folder named `lib`.
- **`yolov8n.pt` is committed** at 6.2 MB — fine under GitHub's 100 MB file limit, and
  it means a clone runs without a weights download. The `# *.pt` lines in `.gitignore`
  are commented out to allow this; uncomment them and add Git LFS if you start
  committing larger checkpoints.
- **`yolo_env/` is 5.1 GB and correctly ignored.** Never force-add it; the wheel is
  aarch64-specific and the tree includes compiled CUDA bindings.
- Commit identity: the initial commit was authored `su <su@local>` from the Pi, which has
  no global `user.name` / `user.email`. That string does not map to a GitHub account, so
  the commit shows as an unlinked author. The repository now sets a local identity
  (`git config --local user.email …` in `.git/config`); run the same on the Pi before
  committing there, or the next board-side commit repeats the problem.

## 5. Missing capabilities, in rough order of payoff

1. **Telemetry.** Nothing records what actually happened: no per-frame CSV, no event
   log, no way to answer "why did it lose me in that doorway". `rover_learning.json`
   is the fossil of an attempt at this.
2. **A tuned-parameters file that is loaded.** Move the CONFIG block into
   `config.json` or a `--config` YAML, read it at startup, and the counters in
   `rover_learning.json` become real again instead of a stale side note.
3. **A single motor module** (`motor.py`) imported by all three scripts, so pins live
   in one place and `person_follow.py` becomes testable without hardware.
4. **Target persistence** — `model.track()` with IoU/ID continuity, or at minimum
   prefer the previous box's track over the raw largest-area pick.
5. **A real proximity stop and speed scaling with box height**, so following distance
   is a behaviour rather than an unreachable constant.
6. **Depth or distance guard for obstacles.** None exists; the robot will drive into a
   wall while a person stays centred in frame.
7. **`detect.py` implementing its own `--interval` and `--skip`** flags, or dropping
   them from the parser.

## 6. Performance

Measured on the Pi 5 with a synthetic random-noise 640×480 frame (`yolov8n.pt`,
torch CPU build, no camera attached — this isolates network cost from capture,
resize, and Python overhead):

| `imgsz` | Steady-state inference | Approx. throughput | First call |
| --- | --- | --- | --- |
| 416 (what `person_follow.py` uses) | 0.059–0.060 s | ≈17 /s | 1.10 s warm-up |
| 640 (what `detect.py` defaults to) | 0.110–0.111 s | ≈9 /s | 0.201 s (already warm) |

Two conclusions. First, `INPUT_SIZE = 416` buys about 1.9× the inference rate of 640,
which is the whole reason the follower does not use the `detect.py` default. Second,
`half=True` in the follower's model call is accepted silently by this CPU-only build
rather than raising — it does nothing useful here, since half precision needs CUDA, but
it is not a bug.

Add capture, the gray-conversion, any CLAHE pass, and box parsing and the real loop
runs below the ≈17 /s ceiling; the script's own `FPS:` line is the number to trust for
a given scene. Note that `detect.py`'s EMA figure measures inference time only, not
end-to-end loop rate, so the two scripts are not directly comparable.

