import numpy as np

class QuadrupedLeg:
    def __init__(self, hip, upper_leg, lower_leg, foot, knee_direction):
        self.hip = hip
        self.upper_leg = upper_leg
        self.lower_leg = lower_leg
        self.foot = foot
        self.knee_direction = knee_direction

class Transformation:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def translate(self, dx, dy, dz):
        self.x += dx
        self.y += dy
        self.z += dz

    def rotate_x(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        new_y = self.y * c - self.z * s
        new_z = self.y * s + self.z * c
        self.y, self.z = new_y, new_z

class Kinematics:
    def __init__(self, quadruped_base):
        self.base = quadruped_base

    def inverse(self, joint_positions, foot_positions):
        calculated_joints = np.zeros(12)

        for i in range(4):
            self.calculate_leg_ik(
                calculated_joints[i * 3:(i + 1) * 3],
                self.base.legs[i],
                foot_positions[i]
            )
            if np.any(np.isnan(calculated_joints[i * 3:(i + 1) * 3])):
                return

        joint_positions[:] = calculated_joints

    @staticmethod
    def calculate_leg_ik(joint_angles, leg, foot_position):
        temp_pos = Transformation(foot_position.x, foot_position.y, foot_position.z)

        l0 = sum([leg.joint_chain[j].y for j in range(1, 4)])
        l1 = -np.sqrt(leg.lower_leg.x**2 + leg.lower_leg.z**2)
        ik_alpha = np.arccos(leg.lower_leg.x / l1) - np.pi / 2

        l2 = -np.sqrt(leg.foot.x**2 + leg.foot.z**2)
        ik_beta = np.arccos(leg.foot.x / l2) - np.pi / 2

        x, y, z = temp_pos.x, temp_pos.y, temp_pos.z
        joint_angles[0] = -(np.arctan(y / z) - ((np.pi / 2) - np.arccos(-l0 / np.sqrt(y**2 + z**2))))

        temp_pos.rotate_x(-joint_angles[0])
        temp_pos.translate(-leg.upper_leg.x, 0.0, -leg.upper_leg.z)
        x, y, z = temp_pos.x, temp_pos.y, temp_pos.z

        target_to_foot = np.sqrt(x**2 + z**2)
        if target_to_foot >= abs(l1) + abs(l2):
            joint_angles[:] = [np.nan, np.nan, np.nan]
            return

        lower_leg_joint = leg.knee_direction * np.arccos((x**2 + z**2 - l1**2 - l2**2) / (2 * l1 * l2))
        upper_leg_joint = (np.arctan(x / z) -
                           np.arctan((l2 * np.sin(lower_leg_joint)) / (l1 + (l2 * np.cos(lower_leg_joint)))))
        lower_leg_joint += ik_beta - ik_alpha
        upper_leg_joint += ik_alpha

        if leg.knee_direction < 0 and upper_leg_joint < 0:
            upper_leg_joint += np.pi
        elif leg.knee_direction > 0 and upper_leg_joint > 0:
            upper_leg_joint += np.pi

        joint_angles[1] = upper_leg_joint
        joint_angles[2] = lower_leg_joint
