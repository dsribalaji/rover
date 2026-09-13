"""One-shot drivetrain check: forward, stop, pivot right, stop.

    ./yolo_env/bin/python motor_test.py

Uses the shared pin map from drivetrain.py on bare gpiozero Motor objects rather than
the controller class, so it also confirms the pins themselves are reachable. Expect
about 5 seconds of motion. Stop early with Ctrl+C.

Exits 0 only if the whole sequence ran; a wiring or GPIO failure exits 1.
"""

import sys
import time

from drivetrain import (
    GPIO_AVAILABLE,
    LEFT_BACKWARD,
    LEFT_ENABLE,
    LEFT_FORWARD,
    RIGHT_BACKWARD,
    RIGHT_ENABLE,
    RIGHT_FORWARD,
)

print("=== Motor Hardware Test ===")

if not GPIO_AVAILABLE:
    print("ERROR: gpiozero is not importable, so no pins can be claimed.")
    sys.exit(1)

from gpiozero import Motor

failed = False
motor_left = None
motor_right = None

try:
    motor_left = Motor(forward=LEFT_FORWARD, backward=LEFT_BACKWARD, enable=LEFT_ENABLE)
    motor_right = Motor(forward=RIGHT_FORWARD, backward=RIGHT_BACKWARD, enable=RIGHT_ENABLE)

    print("1. Moving Forward at 100% speed for 2 seconds...")
    motor_left.forward(1.0)
    motor_right.forward(1.0)
    time.sleep(2)

    print("2. Stopping...")
    motor_left.stop()
    motor_right.stop()
    time.sleep(1)

    print("3. Pivoting Right for 2 seconds...")
    motor_left.forward(1.0)
    motor_right.backward(1.0)
    time.sleep(2)

    print("4. Stopping and cleaning up...")
    print("Test Complete!")

except KeyboardInterrupt:
    print("\nInterrupted - stopping motors.")
    failed = True
except Exception as e:
    print(f"\nERROR: {e}")
    failed = True
finally:
    for motor in (motor_left, motor_right):
        if motor is not None:
            try:
                motor.stop()
                motor.close()
            except Exception as e:
                print(f"ERROR during cleanup: {e}")
                failed = True

sys.exit(1 if failed else 0)
