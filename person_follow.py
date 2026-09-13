import cv2
from ultralytics import YOLO
import os
import signal
import sys
import time

try:
    from gpiozero import Motor
    GPIO_AVAILABLE = True
except (ModuleNotFoundError, ImportError):
    Motor = None
    GPIO_AVAILABLE = False


class TwoChannelMotorController:
    def __init__(self):
        self.speed = 60
        self.available = GPIO_AVAILABLE

        if not self.available:
            print("GPIO libraries not available; motor commands will be logged only.")
            return

        # Left motors (Motor 1 & 2) pins: ENA=12, IN1=23, IN2=24
        self.motor_left = Motor(forward=23, backward=24, enable=12)
        
        # Right motors (Motor 3 & 4) pins: ENB=13, IN3=27, IN4=22
        self.motor_right = Motor(forward=27, backward=22, enable=13)

    def left_motors(self, direction, speed_pct):
        if not self.available: return
        speed = speed_pct / 100.0
        if direction == 'forward': self.motor_left.forward(speed)
        elif direction == 'backward': self.motor_left.backward(speed)
        elif direction == 'stop': self.motor_left.stop()

    def right_motors(self, direction, speed_pct):
        if not self.available: return
        speed = speed_pct / 100.0
        if direction == 'forward': self.motor_right.forward(speed)
        elif direction == 'backward': self.motor_right.backward(speed)
        elif direction == 'stop': self.motor_right.stop()

    def drive(self, left_speed_pct, right_speed_pct):
        if not self.available:
            return

        if left_speed_pct > 0:
            self.left_motors('forward', left_speed_pct)
        else:
            self.left_motors('stop', 0)

        if right_speed_pct > 0:
            self.right_motors('forward', right_speed_pct)
        else:
            self.right_motors('stop', 0)

    def move_forward(self, speed=None):
        if speed is None: speed = self.speed
        self.drive(speed, speed)

    def turn_left(self, speed=None):
        if speed is None: speed = self.speed
        self.left_motors('backward', speed // 2)
        self.right_motors('forward', speed)

    def turn_right(self, speed=None):
        if speed is None: speed = self.speed
        self.left_motors('forward', speed)
        self.right_motors('backward', speed // 2)

    def stop_all(self):
        if not self.available: return
        self.motor_left.stop()
        self.motor_right.stop()

    def cleanup(self):
        if not self.available: return
        self.stop_all()
        self.motor_left.close()
        self.motor_right.close()

# ====================== CONFIG ======================
MODEL_PATH = 'yolov8n.pt'
CONFIDENCE_THRESHOLD = 0.45
CAMERA_SOURCE = '/dev/video0'
FORWARD_SPEED = 80

# Speed-oriented inference settings
INPUT_SIZE = 416
SKIP_FRAMES = 1

# Steering parameters
CENTER_THRESHOLD = 70   # Pixels from center to start turning
TOO_CLOSE_HEIGHT = 600  # High threshold to prevent "stopped (too close)" triggers
STEER_SPEED = 90        # Faster turning
TURN_GAIN = 0.75        # How aggressively to correct lateral offset
MIN_TRACK_SPEED = 35    # Keep both wheels moving while following
STEER_DEADBAND = 0.08   # Ignore tiny frame noise around the center
STEER_DIRECTION_INVERTED = False  # Flip to True only if left/right steering is reversed
SEARCH_HOLD_TIME = 1.5   # Keep moving briefly after the person leaves the frame
SEARCH_TURN_SPEED = 60   # Legacy knob (kept for compatibility)
SEARCH_ALT_START_TIME = 5  # Start alternating scan directions after this long
SEARCH_ALT_INTERVAL = 6    # Increased to make the alternating search arc wider
SEARCH_SCAN_SPEED = 40       # Slow pivot scan speed for low-FPS latency conditions

# Adaptive low-light tuning
ADAPTIVE_LOW_LIGHT = True
LOW_LIGHT_THRESHOLD = 72.0
BRIGHT_LIGHT_THRESHOLD = 145.0
LIGHT_EMA_ALPHA = 0.1
CONF_EMA_ALPHA = 0.15
CAMERA_ADJUST_INTERVAL = 1.0
ENABLE_LOW_LIGHT_ENHANCE = True

# Keep the script usable over SSH/headless setups
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# ====================== LOAD MODEL ======================
print("Loading YOLOv8n model...")
model = YOLO(MODEL_PATH)
model.overrides['imgsz'] = INPUT_SIZE
print(f"Model loaded | Input Size: {INPUT_SIZE}px")

# ====================== CAMERA SETUP ======================
cap = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))

if not cap.isOpened():
    print("Trying fallback camera index 0...")
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Camera could not be opened.")
    sys.exit(1)

print("Camera opened!")


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

# ====================== ROBOT SETUP ======================
robot = TwoChannelMotorController()


def shutdown(*_args):
    print("\nStopping...")
    try:
        robot.stop_all()
        robot.cleanup()
    finally:
        cap.release()
        sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ====================== MAIN LOOP ======================
frame_count = 0
infer_count = 0
start_time = time.time()
last_motion_state = None
last_seen_time = 0.0
last_seen_direction = 0
search_direction = -1
search_last_switch_time = 0.0

print(f"Starting person-follow mode (Input Size={INPUT_SIZE}, Conf={CONFIDENCE_THRESHOLD})")
print("Press Ctrl+C to stop.\n")


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def apply_search_scan(direction):
    if direction > 0:
        robot.turn_right(SEARCH_SCAN_SPEED)
    else:
        robot.turn_left(SEARCH_SCAN_SPEED)


ambient_light_ema = 120.0
detect_conf_ema = 0.50
last_camera_adjust = 0.0

camera_gain = cap.get(cv2.CAP_PROP_GAIN)
camera_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)

if camera_gain != camera_gain or camera_gain < 0:  # NaN or invalid
    camera_gain = 0.0
if camera_exposure != camera_exposure:
    camera_exposure = -6.0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame_count += 1

        if frame_count % SKIP_FRAMES != 0:
            continue

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
            half=True,
        )

        infer_count += 1

        boxes = results[0].boxes
        person_detected = False
        center_offset = 0
        box_height = 0

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
                
                # Calculate center and height
                person_center_x = (x1 + x2) / 2
                box_height = y2 - y1
                center_offset = person_center_x - 320 # 320 is center of 640px wide frame
                
                print(f"Person: Conf={confidence:.2f}, Offset={center_offset:.0f}, Height={box_height:.0f}")
        else:
            detect_conf_ema = (1.0 - CONF_EMA_ALPHA) * detect_conf_ema

        # Movement Logic
        if person_detected:
            if box_height > TOO_CLOSE_HEIGHT:
                robot.stop_all()
                motion_state = "stopped (too close)"
            else:
                frame_center_x = 640 / 2
                normalized_offset = center_offset / frame_center_x

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
                search_direction = last_seen_direction or search_direction
                search_last_switch_time = time.time()
        else:
            time_since_seen = time.time() - last_seen_time

            if last_seen_direction != 0 and time_since_seen <= SEARCH_HOLD_TIME:
                apply_search_scan(last_seen_direction)
                if last_seen_direction > 0:
                    motion_state = "searching right (slow)"
                else:
                    motion_state = "searching left (slow)"
            elif last_seen_direction != 0 and time_since_seen <= SEARCH_ALT_START_TIME:
                apply_search_scan(last_seen_direction)
                if last_seen_direction > 0:
                    motion_state = "searching right (extended slow)"
                else:
                    motion_state = "searching left (extended slow)"
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

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    shutdown()