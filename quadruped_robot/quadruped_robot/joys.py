import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import termios
import sys
import tty

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        self.axes = [0.0] * 8  # Adjust number of axes as needed
        self.buttons = [0] * 12  # Adjust number of buttons as needed
        self.get_logger().info('Initialized KeyboardToJoy node.')

    def get_key(self):
        """Capture a single key press."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def process_key(self, key):
        """Map keypresses to joystick axes and buttons."""
        # Map keys to axes
        if key == 'w':
            self.axes[1] = 1.0  # Forward
        elif key == 's':
            self.axes[1] = -1.0  # Backward
        elif key == 'a':
            self.axes[0] = -1.0  # Left
        elif key == 'd':
            self.axes[0] = 1.0  # Right
        else:
            # Reset axes if no directional key is pressed
            self.axes = [0.0] * len(self.axes)

        # Map keys to buttons
        if key == 'q':
            self.buttons[0] = 1  # Button A
        elif key == 'e':
            self.buttons[1] = 1  # Button B
        elif key == 'm':
            rclpy.shutdown()
        else:
            # Reset all buttons
            self.buttons = [0] * len(self.buttons)

        self.publish_joy()

    def publish_joy(self):
        """Publish the Joy message."""
        joy_msg = Joy()
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons
        self.publisher.publish(joy_msg)
        self.get_logger().info(f'Published Joy: {joy_msg}')

    def run(self):
        """Main loop to capture and process keyboard input."""
        self.get_logger().info('Use keys: [WASD] for axes, [Q/E] for buttons. Press [Ctrl+C] to exit.')
        try:
            while rclpy.ok():
                key = self.get_key()
                self.process_key(key)
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down KeyboardToJoy node.')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
