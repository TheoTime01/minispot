import rclpy
from rclpy.node import Node
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseWithCovarianceStamped
from InverseKineamtics import InverseKinematics

class InvKin_Node(Node):
    def __init__(self):
        self.IK = InverseKinematics()
        self.prev_joint_angs = None
        super().__init__('IK_node')

        # Subscription to the /base_to_footprint_pose topic
        self.sub_ = self.create_subscription(PoseWithCovarianceStamped, '/base_to_footprint_pose', self.sub_callback, 30)

        # Publisher to send joint commands
        self.pub2STM = self.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 30)

        timer_period = 0.02
        self.timerPub = self.create_timer(timer_period, self.pub_callback)
        self.joint_trajectory = None

    def sub_callback(self, msg):
        # Extract position and orientation (quaternion) from PoseWithCovarianceStamped
        eulerAng = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
        # Coordinates for foot positions
        fr_coord = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        fl_coord = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        br_coord = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        bl_coord = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        ang_FR = self.IK.get_FR_joint_angles(fr_coord, eulerAng)
        ang_FL = self.IK.get_FL_joint_angles(fl_coord, eulerAng)
        ang_BR = self.IK.get_BR_joint_angles(br_coord, eulerAng)
        ang_BL = self.IK.get_BL_joint_angles(bl_coord, eulerAng)

        # If angles are valid and not in singularity, convert to degrees
        if not np.any(self.IK.singularity) and np.any(ang_FR != None) and np.any(ang_FL != None) and np.any(ang_BR != None) and np.any(ang_BL != None):
            for i in range(3):
                ang_FR[i] = np.rad2deg(ang_FR[i])
                ang_FL[i] = np.rad2deg(ang_FL[i])
                ang_BR[i] = np.rad2deg(ang_BR[i])
                ang_BL[i] = np.rad2deg(ang_BL[i])

            self.joint_trajectory = JointTrajectory()
            self.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
            self.joint_trajectory.joint_names = [
                "front_left_shoulder", "front_left_leg", "front_left_foot",
                "front_right_shoulder", "front_right_leg", "front_right_foot",
                "rear_left_shoulder", "rear_left_leg", "rear_left_foot",
                "rear_right_shoulder", "rear_right_leg", "rear_right_foot"
            ]

            # Create a JointTrajectoryPoint for the angles
            joint_point = JointTrajectoryPoint()
            joint_point.positions = [
                ang_FR[0], ang_FR[1], ang_FR[1] + ang_FR[2],
                ang_FL[0], ang_FL[1], ang_FL[1] + ang_FL[2],
                ang_BR[0], ang_BR[1], ang_BR[1] + ang_BR[2],
                ang_BL[0], ang_BL[1], ang_BL[1] + ang_BL[2]
            ]
            joint_point.velocities = [0.0] * 12  # Assuming no velocity control for now
            joint_point.effort = [0.0] * 12  # Assuming no effort control for now
            joint_point.time_from_start = rclpy.duration.Duration(seconds=0.5).to_msg()  # 0.5 seconds duration

            self.joint_trajectory.points.append(joint_point)
            self.prev_joint_angs = self.joint_trajectory

        elif self.prev_joint_angs is not None:
            self.joint_trajectory = self.prev_joint_angs

    def pub_callback(self):
        if self.joint_trajectory is not None:
            self.pub2STM.publish(self.joint_trajectory)

def main(args=None):
    rclpy.init(args=args)
    inv_kin = InvKin_Node()
    rclpy.spin(inv_kin)
    inv_kin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
