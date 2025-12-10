import termios
import sys
import math
from pynput import keyboard 
import rclpy
from rclpy.node import Node

from aruku_interfaces.msg import SetWalking

LINEAR_VEL = 35.0  
ANGULAR_VEL_DEG = 20.0 
ANGULAR_VEL_RAD = math.radians(ANGULAR_VEL_DEG)

msg = f"""
Movements:
   W : Forward
   S : Backward
   A : Left
   D : Right
   Q : Turn Left
   E : Turn Right

* Press CTRL+C to exit.
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('gijitaiken_teleop')
        
        self.set_walking_publisher = self.create_publisher(SetWalking, '/walking/set_walking', 10)
        
        self.active_keys = set()
        
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_a = 0.0

    def on_press(self, key):
        try:
            char_key = key.char
            if char_key in ['w', 'a', 's', 'd', 'q', 'e']:
                if char_key not in self.active_keys:
                    self.active_keys.add(char_key)
                    self.process_movement()
        except AttributeError:
            pass 

    def on_release(self, key):
        try:
            char_key = key.char
            if char_key in self.active_keys:
                self.active_keys.remove(char_key)
                self.process_movement()
        except AttributeError:
            pass

    def process_movement(self):
        target_x = 0.0
        target_y = 0.0
        target_a = 0.0
        
        if 'w' in self.active_keys: target_x += LINEAR_VEL
        if 's' in self.active_keys: target_x -= LINEAR_VEL
        
        if 'a' in self.active_keys: target_y += LINEAR_VEL
        if 'd' in self.active_keys: target_y -= LINEAR_VEL
        
        if 'q' in self.active_keys: target_a += ANGULAR_VEL_RAD
        if 'e' in self.active_keys: target_a -= ANGULAR_VEL_RAD

        if (target_x != self.last_x or 
            target_y != self.last_y or 
            target_a != self.last_a):
            
            self.set_walking_command(target_x, target_y, target_a)
            
            self.last_x = target_x
            self.last_y = target_y
            self.last_a = target_a

    def set_walking_command(self, x, y, a):
        msg = SetWalking()

        if x == 0.0 and y == 0.0 and a == 0.0:
            msg.run = False
        else:
            msg.run = True

        msg.x_move = float(x)
        msg.y_move = float(y)
        msg.a_move = float(a)

        self.set_walking_publisher.publish(msg)

def get_terminal_settings():
    return termios.tcgetattr(sys.stdin)

def disable_terminal_echo():
    settings = termios.tcgetattr(sys.stdin)
    settings[3] = settings[3] & ~termios.ECHO
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = get_terminal_settings()
    
    disable_terminal_echo()

    rclpy.init()
    node = TeleopNode()

    print(msg)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.set_walking_command(0.0, 0.0, 0.0)
        except Exception:
            pass 

        if hasattr(node, 'listener'):
            node.listener.stop()
            
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        restore_terminal_settings(settings)

if __name__ == '__main__':
    main()