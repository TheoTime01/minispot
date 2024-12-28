import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import numpy as np

from kinematics import Kinematics, QuadrupedLeg, Transformation


class KinematicsController(Node):
    def __init__(self):
        super().__init__('kinematics_controller')
        self.get_logger().info("Starting Kinematics Controller Node...")

        # Initialize robot structure
        self.base = self.init_robot_base()
        self.kinematics = Kinematics(self.base)
        # Joint mapping
        self.joints_map = {
            "left_front": [
                "front_left_shoulder", "front_left_leg", "front_left_foot"
            ],
            "right_front": [
                "front_right_shoulder", "front_right_leg", "front_right_foot"
            ],
            "left_hind": [
                "rear_left_shoulder", "rear_left_leg", "rear_left_foot"
            ],
            "right_hind": [
                "rear_right_shoulder", "rear_right_leg", "rear_right_foot"
            ],
        }

        # Publisher to effort controller
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_effort_controller/joint_trajectory',
            10
        )


        # Subscriber for foot positions (specific message type: Point)
        self.subscription = self.create_subscription(
            Point,
            '/foot',
            self.foot_callback_point,
            10
        )
        

    def init_robot_base(self):
        leg_length = {
            "hip": np.array([0.1, 0.0, 0.0]),
            "upper_leg": np.array([0.2, 0.0, 0.0]),
            "lower_leg": np.array([0.2, 0.0, 0.0]),
            "foot": np.array([0.05, 0.0, 0.0])
        }
        knee_directions = [1, -1, 1, -1]

        legs = [
            QuadrupedLeg(
                leg_length["hip"], leg_length["upper_leg"], leg_length["lower_leg"], leg_length["foot"], knee_dir
            ) for knee_dir in knee_directions
        ]

        class QuadrupedBase:
            def __init__(self, legs):
                self.legs = legs

        return QuadrupedBase(legs)

    def foot_callback_point(self, msg):
        print("hs")
        # Process geometry_msgs/Point
        foot_positions = [Transformation(msg.x, msg.y, msg.z)] * 4

        joint_positions = np.zeros(12)
        self.kinematics.inverse(joint_positions, foot_positions)

        self.get_logger().info(f"Computed Joint Positions: {joint_positions}")
        self.publish_joint_positions(joint_positions)

    def publish_joint_positions(self, joint_positions):
        joint_data = Float64MultiArray()
        for i, (leg, joints) in enumerate(self.joints_map.items()):
            joint_data.data.extend([
                joint_positions[i * 3],
                joint_positions[i * 3 + 1],
                joint_positions[i * 3 + 2]
            ])
        self.effort_publisher.publish(joint_data)
        self.get_logger().info(f"Published Joint Positions: {joint_data.data}")


def main(args=None):
    rclpy.init(args=args)
    kinematics_controller = KinematicsController()
    rclpy.spin(kinematics_controller)
    kinematics_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
