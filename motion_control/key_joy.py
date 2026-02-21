import sys
import termios # terminal control for linux
import tty # helper function for terminal
import time

from pynput import keyboard

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

axes = [0.0] * 5
buttons = [0] * 10

keys_down = set()
prev_keys_down = set()


# Stores the key pressed as key_down when a key is pressed
def on_press(key):
    try:
        keys_down.add(key.char)
    except AttributeError:
        keys_down.add(str(key))

# removes the key that was just stored when it is no longer pressed
def on_release(key):
    try:
        keys_down.discard(key.char)
    except AttributeError:
        keys_down.discard(str(key))

# listens for keyboard input in the background
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()


class KeyboardJoy(Node):
    def __init__(self):
        super().__init__('keyboard_joy')

        #publishes joy type messages to the joy node
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(1/30, self.update)

        # terminal setup, saves current settings before changing them to turn on cbreak
        # cbreak allows for typing in the terminal without it showing up, so it basically turns off echo
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

        new_settings = termios.tcgetattr(self.fd)
        new_settings[3] = new_settings[3] & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new_settings)

        tty.setcbreak(self.fd) #turns of terminal echo

    def update(self):
        global axes, buttons, keys_down, prev_keys_down

        axes = [0.0] * 5

        # allows for buttons to be toggled if they wernt held by storing what was pressed in the previous frame
        just_pressed = keys_down - prev_keys_down

        # Forward/back
        if 'w' in keys_down:
            axes[1] = 1.0
        elif 's' in keys_down:
            axes[1] = -1.0

        # Left/right
        if 'a' in keys_down:
            axes[0] = 1.0
        elif 'd' in keys_down:
            axes[0] = -1.0

        # Up/down
        if 'Key.space' in keys_down:
            axes[2] = 1.0
        elif 'Key.shift' in keys_down:
            axes[2] = -1.0

        # Yaw
        if 'Key.up' in keys_down:
            axes[3] = 1.0
        elif 'Key.down' in keys_down:
            axes[3] = -1.0

        # Roll
        if 'e' in keys_down:
            axes[4] = 1.0
        elif 'q' in keys_down:
            axes[4] = -1.0

        # thruster toggle (button 0)
        if 't' in just_pressed:
            buttons[3] = 1 - buttons[3]

        if 'g' in just_pressed:
            buttons[0] = 1- buttons[0]

        # slow mode (button 1)
        if '0' in just_pressed:
            buttons[1] = 1 - buttons[1]

        # camera switching buttons mappings

        
        if '1' in just_pressed:
            buttons[4] = 1 - buttons[4]
        

        if '2' in just_pressed:
            buttons[5] = 1 - buttons[5]
   

        if '3' in just_pressed:
            buttons[6] = 1 - buttons[6]
 

        if '4' in just_pressed:
            buttons[7] = 1 - buttons[7]
 

        if '5' in just_pressed:
            buttons[8] = 1 - buttons[8]
   

        # build and publish the joy message
        msg = Joy()
        msg.axes = axes
        msg.buttons = buttons
        self.pub.publish(msg)

        # Update previous keys
        prev_keys_down = set(keys_down)

    def destroy_node(self):
        # restore terminal settings saved earlier so terminal isnt broken
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)

    node = KeyboardJoy()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

