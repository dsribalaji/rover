"""Keyboard teleop for the rover's two-channel drivetrain.

Run from a terminal on the Pi (raw-mode stdin, so a pipe or cron job will not work):

    ./yolo_env/bin/python rover.py

W/S forward/back, A/D gentle turns, Q/E pivots, Space stop, +/- speed trim, X or
Ctrl+C exit. Any unmapped key stops the robot, and so does stdin closing. The pins and
the motor class come from drivetrain.py, shared with person_follow.py and
motor_test.py.
"""

import sys
import tty
import termios

from drivetrain import TwoChannelMotorController


class KeyboardController:
    def __init__(self):
        self.robot = TwoChannelMotorController()
    
    def get_key(self):
        """Get single keypress without waiting for Enter.

        Returns None once stdin is exhausted (SSH dropped, pipe closed), which the
        control loop treats as an exit rather than an unknown key.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch or None
    
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
        if not sys.stdin.isatty():
            print(
                "ERROR: rover.py needs a terminal for raw key input. "
                "Run it directly on the Pi; a pipe, cron job or redirected stdin "
                "will not work.",
                file=sys.stderr,
            )
            return 1

        self.print_controls()
        
        try:
            while True:
                key = self.get_key()

                # Stdin exhausted: the terminal is gone, so do not keep spinning on it.
                if key is None:
                    print("\n\n🛑 Input closed - stopping motors...")
                    self.robot.stop_all()
                    break

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
                
                # Exit. Raw mode clears ISIG, so Ctrl+C reaches us as the 0x03 byte
                # rather than as KeyboardInterrupt — treat it as the exit key here.
                elif key in ['x', 'X', '\x03']:
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
    # run() returns 1 when there is no terminal to read keys from.
    sys.exit(controller.run())