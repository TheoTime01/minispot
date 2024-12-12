import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class LegIKController(Node):
    def __init__(self):
        super().__init__('leg_ik_controller')
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Define leg kinematic parameters (example values, replace with your actual dimensions)
        self.link1_length = 0.1  # Upper leg length
        self.link2_length = 0.1  # Lower leg length

        # Timer for publishing joint commands
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Desired foot positions (example values, replace with trajectory points or inputs)
        desired_positions = {
            'front_left': [0.1, 0.0, -0.2],
            'front_right': [0.1, 0.0, -0.2],
            'rear_left': [-0.1, 0.0, -0.2],
            'rear_right': [-0.1, 0.0, -0.2],
        }

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = [
            'front_left_hip_joint', 'front_left_knee_joint', 'front_left_ankle_joint',
            'front_right_hip_joint', 'front_right_knee_joint', 'front_right_ankle_joint',
            'rear_left_hip_joint', 'rear_left_knee_joint', 'rear_left_ankle_joint',
            'rear_right_hip_joint', 'rear_right_knee_joint', 'rear_right_ankle_joint',
        ]

        joint_states.position = []

        for leg, pos in desired_positions.items():
            hip_angle, knee_angle, ankle_angle = self.inverse_kinematics(pos[0], pos[1], pos[2])
            joint_states.position.extend([hip_angle, knee_angle, ankle_angle])

        self.joint_publisher.publish(joint_states)

    def inverse_kinematics(self, x, y, z):
        """
        Compute the inverse kinematics for a single leg.
        Assumes a 2D planar leg for simplicity.
        """
        d = np.sqrt(x**2 + z**2)
        if d > self.link1_length + self.link2_length:
            self.get_logger().warning("Target out of reach!")
            d = self.link1_length + self.link2_length

        # Compute angles using trigonometry
        cos_knee = (d**2 - self.link1_length**2 - self.link2_length**2) / (2 * self.link1_length * self.link2_length)
        knee_angle = np.arccos(np.clip(cos_knee, -1.0, 1.0))

        sin_knee = np.sqrt(1 - cos_knee**2)
        beta = np.arctan2(z, x)
        alpha = np.arctan2(self.link2_length * sin_knee, self.link1_length + self.link2_length * cos_knee)

        hip_angle = beta - alpha
        ankle_angle = -knee_angle - hip_angle

        return hip_angle, knee_angle, ankle_angle


def main(args=None):
    rclpy.init(args=args)
    node = LegIKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
