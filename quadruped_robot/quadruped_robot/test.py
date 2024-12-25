import rclpy
from rclpy.node import Node
from hyperdog_msgs.msg import JoyCtrlCmds
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Create a publisher to publish control commands
        self.publisher = self.create_publisher(JoyCtrlCmds, '/hyperdog_joy_ctrl_cmd', 10)

        # Timer to continuously send commands (5 Hz)
        self.timer = self.create_timer(0.2, self.publish_commands)

        # Initialize the control message
        self.cmd = JoyCtrlCmds()

        # Start robot in standing state
        self.cmd.states[0] = True  # Start the robot
        self.cmd.states[1] = False  # Set walk state to False initially
        self.cmd.states[2] = False  # Set side move mode to False
        self.cmd.gait_type = 1  # Choose gait type 1 (or any other based on the robot configuration)
        self.cmd.pose.position.z = 0.2  # Set a starting height (stand up position)
        self.cmd.pose.orientation.w = 1.0  # Keep orientation neutral

        self.get_logger().info("Robot control node initialized")

    def publish_commands(self):
        # Transition to walking state after standing
        if self.cmd.states[0] and not self.cmd.states[1]:
            self.cmd.states[1] = True  # Start walking

        # Set continuous gait parameters for walking
        self.cmd.gait_step.x = 0.5  # Step length in X direction
        self.cmd.gait_step.y = 0.0  # Step length in Y direction (no sideways movement)
        self.cmd.gait_step.z = 0.1  # Step height (adjust based on gait configuration)

        # Continuously publish the command to move the robot
        self.publisher.publish(self.cmd)
        self.get_logger().info("Publishing walking command")

def main(args=None):
    rclpy.init(args=args)

    robot_control_node = RobotControlNode()

    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
