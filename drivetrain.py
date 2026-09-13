"""Shared L298N drivetrain for the rover: pin map and the two-channel controller.

Every script that moves the robot (rover.py, person_follow.py, motor_test.py) gets its
pins and its motor class from here, so a rewiring is a one-file change. The BCM-to-
header-pins table and the 0-100 percent speed convention are in docs/hardware.md.

With gpiozero missing, the controller still works and only logs what it would have
sent — useful for reading the code on a laptop, never as a substitute for a bench test.
"""

LEFT_FORWARD = 23    # IN1
LEFT_BACKWARD = 24   # IN2
LEFT_ENABLE = 12     # ENA (hardware PWM on the Pi 5)

RIGHT_FORWARD = 27   # IN3
RIGHT_BACKWARD = 22  # IN4
RIGHT_ENABLE = 13    # ENB (hardware PWM on the Pi 5)

MIN_SPEED = 20
MAX_SPEED = 100
SPEED_STEP = 10

try:
    from gpiozero import Motor

    GPIO_AVAILABLE = True
except (ModuleNotFoundError, ImportError):
    Motor = None
    GPIO_AVAILABLE = False


class TwoChannelMotorController:
    """Drives both channels of an L298N; percentages are 0-100, negative is reverse."""

    def __init__(self):
        self.speed = 60
        self.available = GPIO_AVAILABLE

        if not self.available:
            print("GPIO libraries not available; motor commands will be logged only.")
            return

        try:
            self.motor_left = Motor(
                forward=LEFT_FORWARD,
                backward=LEFT_BACKWARD,
                enable=LEFT_ENABLE,
            )
            self.motor_right = Motor(
                forward=RIGHT_FORWARD,
                backward=RIGHT_BACKWARD,
                enable=RIGHT_ENABLE,
            )
        except Exception as exc:
            # Restarting too soon after another run lands here as lgpio's "GPIO busy".
            # Release whatever was claimed, or the next start fails the same way.
            if getattr(self, "motor_left", None) is not None:
                self.motor_left.close()
            self.available = False
            raise RuntimeError(
                f"Could not claim the drivetrain pins (BCM "
                f"{LEFT_FORWARD}/{LEFT_BACKWARD}/{LEFT_ENABLE} and "
                f"{RIGHT_FORWARD}/{RIGHT_BACKWARD}/{RIGHT_ENABLE}): {exc}. Another process "
                f"usually holds them - stop the other rover script, or wait a second "
                f"after it exits, and start again."
            ) from exc

    def _side(self, motor, direction, speed_pct):
        speed = abs(speed_pct) / 100.0
        if direction == 'forward':
            motor.forward(speed)
        elif direction == 'backward':
            motor.backward(speed)
        else:
            motor.stop()

    def left_motors(self, direction, speed_pct):
        if not self.available:
            return
        self._side(self.motor_left, direction, speed_pct)

    def right_motors(self, direction, speed_pct):
        if not self.available:
            return
        self._side(self.motor_right, direction, speed_pct)

    def drive(self, left_speed_pct, right_speed_pct):
        """Set both sides; a negative percentage commands reverse on that side."""
        if not self.available:
            return

        for motors, pct in ((self.left_motors, left_speed_pct),
                            (self.right_motors, right_speed_pct)):
            if pct > 0:
                motors('forward', pct)
            elif pct < 0:
                motors('backward', pct)
            else:
                motors('stop', 0)

    def move_forward(self, speed=None):
        if speed is None:
            speed = self.speed
        self.drive(speed, speed)

    def move_backward(self, speed=None):
        if speed is None:
            speed = self.speed
        self.drive(-speed, -speed)

    def turn_left(self, speed=None):
        """Gentle left: left side backs up, right side forwards."""
        if speed is None:
            speed = self.speed
        self.drive(-(speed // 2), speed)

    def turn_right(self, speed=None):
        """Gentle right: left side forwards, right side backs up."""
        if speed is None:
            speed = self.speed
        self.drive(speed, -(speed // 2))

    def pivot_left(self, speed=None):
        """Spin in place to the left."""
        if speed is None:
            speed = self.speed
        self.drive(-speed, speed)

    def pivot_right(self, speed=None):
        """Spin in place to the right."""
        if speed is None:
            speed = self.speed
        self.drive(speed, -speed)

    def stop_all(self):
        if not self.available:
            return
        self.motor_left.stop()
        self.motor_right.stop()

    def increase_speed(self):
        """Raise the default speed by SPEED_STEP and return it."""
        self.speed = min(MAX_SPEED, self.speed + SPEED_STEP)
        return self.speed

    def decrease_speed(self):
        """Lower the default speed by SPEED_STEP and return it."""
        self.speed = max(MIN_SPEED, self.speed - SPEED_STEP)
        return self.speed

    def cleanup(self):
        if not self.available:
            return
        try:
            self.stop_all()
        finally:
            self.motor_left.close()
            self.motor_right.close()
            # Every motor method bails out on `available`, so clearing it here makes the
            # controller inert. Without this, a control loop still mid-iteration when a
            # signal handler cleans up hits closed pins and raises GPIODeviceClosed.
            self.available = False
