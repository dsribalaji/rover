"""Autonomous person-following loop: YOLOv8n detection drives differential steering.

Run from the repository root on the Pi (MODEL_PATH is relative):

    ./yolo_env/bin/python person_follow.py

Has no command-line arguments; everything is configured in the CONFIG block below.
Behaviour, the steering maths, and the lost-target search ladder are described in
docs/architecture.md; each knob is listed with its current value in docs/tuning.md.
Nothing is opened or claimed until this file is run as a script, so the CONFIG block
and the steering maths can be imported on a laptop without a camera or GPIO.
"""

import cv2
from ultralytics import YOLO
import signal
import sys
import threading
import time

from drivetrain import TwoChannelMotorController, GPIO_AVAILABLE


# ====================== CONFIG ======================
MODEL_PATH = 'yolov8n.pt'
CONFIDENCE_THRESHOLD = 0.45
CAMERA_SOURCE = '/dev/video0'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FORWARD_SPEED = 80

# Speed-oriented inference settings
INPUT_SIZE = 416
FRAME_SKIP = 0           # 0 = infer on every frame, N = infer every (N+1)th frame
# Must stay longer than one inference cycle, or the watchdog stops the drivetrain
# between commands. Measured on the Pi 5: median 200ms, p95 250ms per inference.
MOTOR_WATCHDOG_S = 0.75  # Stop the drivetrain if the loop stops issuing commands

# Steering parameters
MOUNT_OFFSET_PX = 0      # Calibration: paste the mean `Offset=` the loop prints when a
                         # person stands dead ahead (-73 px on the 2026-09-13 bench mount)
# Proximity stop, as a share of frame height so it survives a resolution change. The
# current low, upward-tilted mount fills the frame with a person from ~1m out (measured
# median box height 459 of 480 rows), so this only fires when they are almost touching
# the rover. Remount the camera higher and tilted down, then tighten this towards 0.85.
TOO_CLOSE_HEIGHT_RATIO = 0.99
TURN_GAIN = 0.75         # How aggressively to correct lateral offset
MIN_TRACK_SPEED = 35     # Keep both wheels moving while following
STEER_DEADBAND = 0.08    # Ignore tiny frame noise around the center
STEER_DIRECTION_INVERTED = False  # Flip to True only if left/right steering is reversed
SEARCH_ALT_START_TIME = 5    # Start alternating scan directions after this long
SEARCH_ALT_INTERVAL = 6      # Width of the alternating search arc
SEARCH_SCAN_SPEED = 40       # Slow pivot scan speed for low-FPS latency conditions
SEARCH_HOLD_TIME = 1.5       # Search in the last seen direction until this long

# Adaptive low-light tuning
ADAPTIVE_LOW_LIGHT = True
LOW_LIGHT_THRESHOLD = 72.0
BRIGHT_LIGHT_THRESHOLD = 145.0
LIGHT_EMA_ALPHA = 0.1
CONF_EMA_ALPHA = 0.15
CAMERA_ADJUST_INTERVAL = 1.0
ENABLE_LOW_LIGHT_ENHANCE = True


# ====================== RUNTIME STATE ======================
# Shared with the watchdog thread and the signal handlers, so they stay at module level.
model = None
cap = None
robot = None
running = False
_last_beat = 0.0
_watchdog_stop = threading.Event()
_shutting_down = False


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _beat():
    """Stamp the control loop as alive, which arms the motor watchdog."""
    global _last_beat
    _last_beat = time.time()


def motor_watchdog():
    stalled = False
    while not _watchdog_stop.wait(0.1):
        if time.time() - _last_beat > MOTOR_WATCHDOG_S:
            # Keep retrying: a watchdog that dies on the first failed stop has stopped
            # being the safety net at exactly the moment it is needed.
            try:
                robot.stop_all()
            except Exception as exc:
                if not stalled:
                    print(f">>> WATCHDOG: stop_all() failed: {exc}")
            else:
                if not stalled:
                    print(
                        f">>> WATCHDOG: drivetrain stopped, no control-loop command for "
                        f"{MOTOR_WATCHDOG_S:.2f}s"
                    )
            stalled = True
        else:
            stalled = False


def enhance_low_light(input_frame):
    ycrcb = cv2.cvtColor(input_frame, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    y = clahe.apply(y)
    enhanced = cv2.merge((y, cr, cb))
    return cv2.cvtColor(enhanced, cv2.COLOR_YCrCb2BGR)


def get_exposure_limits(current_exposure):
    # Many V4L2 cameras expose manual exposure as negative values.
    if current_exposure <= 0:
        return -13.0, -1.0, 1.0
    return 1.0, 500.0, 5.0


def adapt_camera(cap_obj, light_ema, conf_ema, gain_value, exposure_value):
    gain_min, gain_max, gain_step = 0.0, 64.0, 2.0
    exp_min, exp_max, exp_step = get_exposure_limits(exposure_value)

    if light_ema < LOW_LIGHT_THRESHOLD and conf_ema < 0.65:
        gain_value = clamp(gain_value + gain_step, gain_min, gain_max)
        exposure_value = clamp(exposure_value + exp_step, exp_min, exp_max)
    elif light_ema > BRIGHT_LIGHT_THRESHOLD:
        gain_value = clamp(gain_value - gain_step, gain_min, gain_max)
        exposure_value = clamp(exposure_value - exp_step, exp_min, exp_max)
    else:
        return gain_value, exposure_value

    gain_ok = cap_obj.set(cv2.CAP_PROP_GAIN, gain_value)
    exposure_ok = cap_obj.set(cv2.CAP_PROP_EXPOSURE, exposure_value)

    if gain_ok or exposure_ok:
        print(
            f"Camera tune | Light={light_ema:.0f} Conf={conf_ema:.2f} "
            f"Gain={gain_value:.1f} Exp={exposure_value:.1f}"
        )

    return gain_value, exposure_value


def apply_search_scan(direction):
    if direction > 0:
        robot.turn_right(SEARCH_SCAN_SPEED)
    else:
        robot.turn_left(SEARCH_SCAN_SPEED)


def shutdown(*_args):
    """Stop the drivetrain and release the camera. Safe to call more than once."""
    global running, _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
    running = False

    print("\nStopping...")
    _watchdog_stop.set()
    try:
        if robot is not None:
            robot.cleanup()
    finally:
        if cap is not None:
            cap.release()


def install_signal_handlers():
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)


# ====================== SETUP ======================
def load_model():
    print("Loading YOLOv8n model...")
    loaded = YOLO(MODEL_PATH)
    loaded.overrides['imgsz'] = INPUT_SIZE
    print(f"Model loaded | Input Size: {INPUT_SIZE}px")
    return loaded


def open_camera():
    opened = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_V4L2)

    if not opened.isOpened():
        # Release the failed handle: /dev/video0 stays claimed until then, which makes
        # the fallback just as likely to fail.
        opened.release()
        print("Trying fallback camera index 0...")
        opened = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not opened.isOpened():
        print("Camera could not be opened.")
        return None

    # Configured after the open attempts, so the fallback path gets the same settings.
    opened.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    opened.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    opened.set(cv2.CAP_PROP_FPS, 30)
    opened.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    opened.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'),
    )
    return opened



# ====================== MAIN LOOP ======================
def follow_person():
    frame_count = 0
    infer_count = 0
    read_failures = 0
    start_time = time.time()
    last_motion_state = None
    last_seen_time = 0.0
    last_seen_direction = 0
    search_direction = -1
    search_last_switch_time = 0.0

    ambient_light_ema = 120.0
    detect_conf_ema = 0.50
    last_camera_adjust = 0.0

    camera_gain = cap.get(cv2.CAP_PROP_GAIN)
    camera_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)

    if camera_gain != camera_gain or camera_gain < 0:  # NaN or invalid
        camera_gain = 0.0
    if camera_exposure != camera_exposure:
        camera_exposure = -6.0

    watchdog = threading.Thread(target=motor_watchdog, daemon=True)
    watchdog.start()

    try:
        while running:
            frame_count += 1
            ret, frame = cap.read()
            if not ret:
                # Camera dropped: hold the drivetrain stopped instead of spinning here.
                read_failures += 1
                if read_failures == 1 or read_failures % 20 == 0:
                    print(f"WARNING: no frame from {CAMERA_SOURCE} ({read_failures} in a row)")
                robot.stop_all()
                _beat()
                time.sleep(0.05)
                continue
            read_failures = 0

            # FRAME_SKIP = 0 infers on every frame; N infers on the first frame and
            # then every (N+1)th, so the first frame is never thrown away.
            if FRAME_SKIP > 0 and (frame_count - 1) % (FRAME_SKIP + 1) != 0:
                # Skipped frames leave the previous drivetrain command in force, but the
                # loop is demonstrably alive — beat, or a high FRAME_SKIP trips the watchdog.
                _beat()
                continue

            frame_height, frame_width = frame.shape[:2]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_light = float(cv2.mean(gray)[0])
            ambient_light_ema = (
                (1.0 - LIGHT_EMA_ALPHA) * ambient_light_ema + LIGHT_EMA_ALPHA * frame_light
            )

            if ENABLE_LOW_LIGHT_ENHANCE and ambient_light_ema < LOW_LIGHT_THRESHOLD:
                infer_frame = enhance_low_light(frame)
            else:
                infer_frame = frame

            if ADAPTIVE_LOW_LIGHT and (time.time() - last_camera_adjust) >= CAMERA_ADJUST_INTERVAL:
                camera_gain, camera_exposure = adapt_camera(
                    cap,
                    ambient_light_ema,
                    detect_conf_ema,
                    camera_gain,
                    camera_exposure,
                )
                last_camera_adjust = time.time()

            results = model(
                infer_frame,
                verbose=False,
                conf=CONFIDENCE_THRESHOLD,
                imgsz=INPUT_SIZE,
            )

            infer_count += 1

            boxes = results[0].boxes
            person_detected = False
            center_offset = 0.0
            box_height = 0.0

            if boxes is not None and len(boxes) > 0:
                # Find the largest person (likely the one to follow)
                max_area = 0
                best_box = None

                for box in boxes:
                    class_id = int(box.cls[0])
                    class_name = model.names[class_id]

                    if class_name == 'person':
                        x1, y1, x2, y2 = box.xyxy[0]
                        area = (x2 - x1) * (y2 - y1)
                        if area > max_area:
                            max_area = area
                            best_box = box

                if best_box is not None:
                    person_detected = True
                    x1, y1, x2, y2 = best_box.xyxy[0]
                    confidence = float(best_box.conf[0])
                    detect_conf_ema = (
                        (1.0 - CONF_EMA_ALPHA) * detect_conf_ema + CONF_EMA_ALPHA * confidence
                    )

                    # Offsets are relative to the actual frame, not an assumed resolution.
                    person_center_x = (x1 + x2) / 2
                    box_height = y2 - y1
                    center_offset = person_center_x - (frame_width / 2) - MOUNT_OFFSET_PX

                    print(
                        f"Person: Conf={confidence:.2f}, Offset={center_offset:.0f}, "
                        f"Height={box_height:.0f}"
                    )
            else:
                detect_conf_ema = (1.0 - CONF_EMA_ALPHA) * detect_conf_ema

            # Movement Logic
            if person_detected:
                if box_height > TOO_CLOSE_HEIGHT_RATIO * frame_height:
                    robot.stop_all()
                    motion_state = "stopped (too close)"
                else:
                    normalized_offset = center_offset / (frame_width / 2)

                    if abs(normalized_offset) < STEER_DEADBAND:
                        steer = 0.0
                    else:
                        steer = clamp(normalized_offset * TURN_GAIN, -0.6, 0.6)

                    if STEER_DIRECTION_INVERTED:
                        steer *= -1.0

                    left_speed = clamp(FORWARD_SPEED * (1.0 + steer), MIN_TRACK_SPEED, 100)
                    right_speed = clamp(FORWARD_SPEED * (1.0 - steer), MIN_TRACK_SPEED, 100)

                    robot.drive(left_speed, right_speed)

                    if steer > 0:
                        direction = "right"
                    elif steer < 0:
                        direction = "left"
                    else:
                        direction = "straight"

                    motion_state = (
                        f"following {direction} | left={left_speed:.0f}% right={right_speed:.0f}%"
                    )
                    last_seen_time = time.time()
                    last_seen_direction = 1 if steer > 0 else -1 if steer < 0 else 0
                    if last_seen_direction != 0:
                        search_direction = last_seen_direction
                    search_last_switch_time = time.time()
            else:
                time_since_seen = time.time() - last_seen_time

                if last_seen_direction != 0 and time_since_seen <= SEARCH_ALT_START_TIME:
                    apply_search_scan(last_seen_direction)
                    side = "right" if last_seen_direction > 0 else "left"
                    phase = "hold" if time_since_seen <= SEARCH_HOLD_TIME else "extended"
                    motion_state = f"searching {side} ({phase} slow)"
                elif last_seen_direction != 0:
                    current_time = time.time()
                    if search_last_switch_time == 0.0:
                        search_last_switch_time = current_time

                    if current_time - search_last_switch_time >= SEARCH_ALT_INTERVAL:
                        search_direction *= -1
                        search_last_switch_time = current_time

                    apply_search_scan(search_direction)
                    if search_direction > 0:
                        motion_state = "alternating search right (slow)"
                    else:
                        motion_state = "alternating search left (slow)"
                else:
                    robot.stop_all()
                    motion_state = "stopped (no person)"
                    last_seen_direction = 0
                    search_direction = -1
                    search_last_switch_time = 0.0

            # The loop is alive and has just commanded the drivetrain.
            _beat()

            if motion_state != last_motion_state:
                print(f">>> ROBOT ACTION: {motion_state}")
                last_motion_state = motion_state

            if infer_count % 20 == 0:
                elapsed = time.time() - start_time
                fps = infer_count / elapsed if elapsed > 0 else 0.0
                print(
                    f"FPS: {fps:.1f} | Light={ambient_light_ema:.0f} | "
                    f"ConfEMA={detect_conf_ema:.2f}"
                )
    finally:
        _watchdog_stop.set()
        watchdog.join(timeout=1.0)


def main():
    global model, cap, robot, running

    # Handlers first, so Ctrl+C during model load or camera open still cleans up.
    install_signal_handlers()

    robot = TwoChannelMotorController()
    if not GPIO_AVAILABLE:
        print(
            "\n*** SIMULATION MODE: no GPIO, the printed ROBOT ACTION lines are the "
            "only thing happening. ***\n"
        )
    model = load_model()
    cap = open_camera()
    if cap is None:
        robot.cleanup()
        return 1

    print("Camera opened!")
    print(f"Starting person-follow mode (Input Size={INPUT_SIZE}, Conf={CONFIDENCE_THRESHOLD})")
    print("Press Ctrl+C to stop.\n")

    running = True
    _beat()
    follow_person()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # Only reachable from a Ctrl+C before the signal handlers were installed.
        print("\nStopped by user.")
        sys.exit(0)
