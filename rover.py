"""Keyboard teleop for the rover's two-channel drivetrain.

Run from a terminal on the Pi (raw-mode stdin, so a pipe or cron job will not work):

    ./yolo_env/bin/python rover.py

W/S forward/back, A/D gentle turns, Q/E pivots, Space stop, +/- speed trim, X exit.
Any unmapped key stops the robot. Wiring and the 0-100 percent speed convention are
documented in docs/hardware.md; the same pin set appears in person_follow.py and
motor_test.py.
"""

from gpiozero import Motor
import time
import sys
import tty
import termios

class TwoChannelMotorController:
    def __init__(self):
        # Left motors (Motor 1 & 2) pins
        # ENA=12, IN1=23, IN2=24
        self.motor_left = Motor(forward=23, backward=24, enable=12)
        
        # Right motors (Motor 3 & 4) pins
        # ENB=13, IN3=27, IN4=22
        self.motor_right = Motor(forward=27, backward=22, enable=13)
        
        # Default speed (0.0 to 1.0 in gpiozero)
        self.speed = 60
    
    def left_motors(self, direction, speed):
        """Control left side motors (1 & 2)"""
        speed_val = speed / 100.0
        if direction == 'forward':
            self.motor_left.forward(speed_val)
        elif direction == 'backward':
            self.motor_left.backward(speed_val)
        elif direction == 'stop':
            self.motor_left.stop()
    
    def right_motors(self, direction, speed):
        """Control right side motors (3 & 4)"""
        speed_val = speed / 100.0
        if direction == 'forward':
            self.motor_right.forward(speed_val)
        elif direction == 'backward':
            self.motor_right.backward(speed_val)
        elif direction == 'stop':
            self.motor_right.stop()
    
    def move_forward(self, speed=None):
        """Move robot forward"""
        if speed is None:
            speed = self.speed
        self.left_motors('forward', speed)
        self.right_motors('forward', speed)
    
    def move_backward(self, speed=None):
        """Move robot backward"""
        if speed is None:
            speed = self.speed
        self.left_motors('backward', speed)
        self.right_motors('backward', speed)
    
    def turn_left(self, speed=None):
        """Turn left (left motors stop/slow, right forward)"""
        if speed is None:
            speed = self.speed
        self.left_motors('backward', speed // 2)
        self.right_motors('forward', speed)
    
    def turn_right(self, speed=None):
        """Turn right (left motors forward, right stop/slow)"""
        if speed is None:
            speed = self.speed
        self.left_motors('forward', speed)
        self.right_motors('backward', speed // 2)
    
    def pivot_left(self, speed=None):
        """Pivot left in place (left backward, right forward)"""
        if speed is None:
            speed = self.speed
        self.left_motors('backward', speed)
        self.right_motors('forward', speed)
    
    def pivot_right(self, speed=None):
        """Pivot right in place (left forward, right backward)"""
        if speed is None:
            speed = self.speed
        self.left_motors('forward', speed)
        self.right_motors('backward', speed)
    
    def stop_all(self):
        """Stop all motors"""
        self.motor_left.stop()
        self.motor_right.stop()
    
    def increase_speed(self):
        """Increase speed by 10%"""
        self.speed = min(100, self.speed + 10)
        return self.speed
    
    def decrease_speed(self):
        """Decrease speed by 10%"""
        self.speed = max(20, self.speed - 10)
        return self.speed
    
    def cleanup(self):
        """Clean up"""
        self.stop_all()
        self.motor_left.close()
        self.motor_right.close()


class KeyboardController:
    def __init__(self):
        self.robot = TwoChannelMotorController()
    
    def get_key(self):
        """Get single keypress without waiting for Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def print_controls(self):
        """Display control instructions"""
        print("\n" + "="*50)
        print("  🤖 ROBOT MOTOR CONTROLLER")
        print("="*50)
        print("\n📋 CONTROLS:")
        print("  W / w  →  Move Forward")
        print("  S / s  →  Move Backward")
        print("  A / a  →  Turn Left (gentle)")
        print("  D / d  →  Turn Right (gentle)")
        print("  Q / q  →  Pivot Left (sharp turn)")
        print("  E / e  →  Pivot Right (sharp turn)")
        print("  SPACE  →  STOP (Emergency)")
        print("  + / =  →  Increase Speed")
        print("  - / _  →  Decrease Speed")
        print("  X / x  →  EXIT Program")
        print("\n" + "="*50)
        print(f"Current Speed: {self.robot.speed}%")
        print("="*50 + "\n")
        print("Ready! Press keys to control...")
    
    def update_status(self, action):
        """Print current action and speed"""
        print(f"\r[Speed: {self.robot.speed:3d}%] {action:<30}", end='', flush=True)
    
    def run(self):
        """Main control loop"""
        self.print_controls()
        
        try:
            while True:
                key = self.get_key()
                
                # Forward
                if key in ['w', 'W']:
                    self.robot.move_forward()
                    self.update_status("⬆️  Moving FORWARD")
                
                # Backward
                elif key in ['s', 'S']:
                    self.robot.move_backward()
                    self.update_status("⬇️  Moving BACKWARD")
                
                # Turn Left (gentle)
                elif key in ['a', 'A']:
                    self.robot.turn_left()
                    self.update_status("⬅️  Turning LEFT")
                
                # Turn Right (gentle)
                elif key in ['d', 'D']:
                    self.robot.turn_right()
                    self.update_status("➡️  Turning RIGHT")
                
                # Pivot Left (sharp)
                elif key in ['q', 'Q']:
                    self.robot.pivot_left()
                    self.update_status("↩️  Pivoting LEFT (sharp)")
                
                # Pivot Right (sharp)
                elif key in ['e', 'E']:
                    self.robot.pivot_right()
                    self.update_status("↪️  Pivoting RIGHT (sharp)")
                
                # Stop
                elif key == ' ':
                    self.robot.stop_all()
                    self.update_status("🛑 STOPPED")
                
                # Increase speed
                elif key in ['+', '=']:
                    new_speed = self.robot.increase_speed()
                    self.update_status(f"⬆️  Speed increased to {new_speed}%")
                
                # Decrease speed
                elif key in ['-', '_']:
                    new_speed = self.robot.decrease_speed()
                    self.update_status(f"⬇️  Speed decreased to {new_speed}%")
                
                # Exit
                elif key in ['x', 'X']:
                    print("\n\n🛑 Stopping motors and exiting...")
                    self.robot.stop_all()
                    break
                
                # Unknown key
                else:
                    # Stop on any other key for safety
                    self.robot.stop_all()
                    self.update_status("⚠️  Unknown key - STOPPED")
        
        except KeyboardInterrupt:
            print("\n\n⚠️  Keyboard interrupt detected!")
        
        finally:
            print("\n🧹 Cleaning up GPIO...")
            self.robot.cleanup()
            print("✅ Done! Goodbye!\n")


if __name__ == "__main__":
    controller = KeyboardController()
    controller.run()