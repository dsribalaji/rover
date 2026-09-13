"""One-shot drivetrain check: forward, stop, pivot right, stop.

    ./yolo_env/bin/python motor_test.py

Uses the same BCM pins as rover.py and person_follow.py, on bare gpiozero Motor
objects rather than the controller class, so it also confirms the pins themselves are
reachable. Expect about 5 seconds of motion. Stop early with Ctrl+C.
"""

from gpiozero import Motor
import time
import sys

print("=== Motor Hardware Test ===")
try:
    # Using the same pins as your rover.py and person_follow.py
    motor_left = Motor(forward=23, backward=24, enable=12)
    motor_right = Motor(forward=27, backward=22, enable=13)
    
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
    
    print("4. Stopping and Cleaning up...")
    motor_left.stop()
    motor_right.stop()
    
    print("Test Complete!")
    
except Exception as e:
    print(f"\nERROR: {e}")
finally:
    sys.exit()
