import numpy as np
from dataclasses import dataclass
import math
from typing import List, Tuple

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class QuadrupedLeg:
    hip: Vector3
    upper_leg: Vector3
    lower_leg: Vector3
    foot: Vector3
    knee_direction: float  # 1 or -1 for knee direction

class Transformation:
    def __init__(self):
        self.matrix = np.eye(4)
    
    def translate(self, x: float, y: float, z: float) -> None:
        translation = np.eye(4)
        translation[0:3, 3] = [x, y, z]
        self.matrix = self.matrix @ translation
    
    def rotate_x(self, angle: float) -> None:
        rotation = np.eye(4)
        c = np.cos(angle)
        s = np.sin(angle)
        rotation[1:3, 1:3] = [[c, -s], [s, c]]
        self.matrix = self.matrix @ rotation
    
    def rotate_y(self, angle: float) -> None:
        rotation = np.eye(4)
        c = np.cos(angle)
        s = np.sin(angle)
        rotation[0:3:2, 0:3:2] = [[c, s], [-s, c]]
        self.matrix = self.matrix @ rotation

    def get_position(self) -> Tuple[float, float, float]:
        return self.matrix[0:3, 3]
    
    def __str__(self):
        x, y, z = self.get_position()
        return f"Position(x={x:.3f}, y={y:.3f}, z={z:.3f})"

class SpotMicroKinematics:
    def __init__(self, legs: List[QuadrupedLeg], debug=False):
        self.legs = legs
        self.EPSILON = 1e-10
        self.debug = debug

    def debug_print(self, *args):
        if self.debug:
            print(*args)

    def safe_arccos(self, value: float) -> float:
        if value > 1.0:
            self.debug_print(f"arccos value clamped from {value} to 1.0")
            return np.arccos(1.0)
        elif value < -1.0:
            self.debug_print(f"arccos value clamped from {value} to -1.0")
            return np.arccos(-1.0)
        return np.arccos(value)

    def safe_divide(self, numerator: float, denominator: float) -> float:
        if abs(denominator) < self.EPSILON:
            self.debug_print(f"Division by near-zero value: {denominator}")
            return float('nan')
        return numerator / denominator

    def inverse_kinematics_single_leg(self, leg: QuadrupedLeg, foot_position: Transformation) -> Tuple[float, float, float]:
        try:
            # Get foot position coordinates
            x, y, z = foot_position.get_position()
            self.debug_print(f"\nCalculating IK for foot position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            # Calculate l0 (sum of y components)
            l0 = sum(joint.y for joint in [leg.hip, leg.upper_leg, leg.lower_leg, leg.foot])
            self.debug_print(f"l0 (total y offset): {l0}")
            
            # Calculate l1 and l2 (lengths of leg segments)
            l1 = -np.sqrt(leg.lower_leg.x**2 + leg.lower_leg.z**2)
            l2 = -np.sqrt(leg.foot.x**2 + leg.foot.z**2)
            self.debug_print(f"l1 (lower leg length): {l1}")
            self.debug_print(f"l2 (foot length): {l2}")
            
            if abs(l1) < self.EPSILON or abs(l2) < self.EPSILON:
                self.debug_print("Invalid leg lengths")
                return float('nan'), float('nan'), float('nan')
            
            # Calculate intermediate angles
            lower_leg_ratio = self.safe_divide(leg.lower_leg.x, l1)
            foot_ratio = self.safe_divide(leg.foot.x, l2)
            
            if np.isnan(lower_leg_ratio) or np.isnan(foot_ratio):
                self.debug_print("Invalid leg ratios")
                return float('nan'), float('nan'), float('nan')
            
            ik_alpha = self.safe_arccos(lower_leg_ratio) - (np.pi / 2)
            ik_beta = self.safe_arccos(foot_ratio) - (np.pi / 2)
            self.debug_print(f"ik_alpha: {np.degrees(ik_alpha):.2f}°")
            self.debug_print(f"ik_beta: {np.degrees(ik_beta):.2f}°")
            
            # Calculate hip joint angle
            y_z_norm = np.sqrt(y**2 + z**2)
            if y_z_norm < self.EPSILON:
                self.debug_print("Invalid y-z norm")
                return float('nan'), float('nan'), float('nan')
            
            l0_ratio = self.safe_divide(-l0, y_z_norm)
            if abs(l0_ratio) > 1:
                self.debug_print(f"Invalid l0 ratio: {l0_ratio}")
                return float('nan'), float('nan'), float('nan')
            
            hip_joint = -(np.arctan2(y, z) - ((np.pi/2) - self.safe_arccos(l0_ratio)))
            self.debug_print(f"hip_joint: {np.degrees(hip_joint):.2f}°")
            
            # Transform foot position
            temp_pos = Transformation()
            temp_pos.matrix = foot_position.matrix.copy()
            temp_pos.rotate_x(-hip_joint)
            temp_pos.translate(-leg.upper_leg.x, 0.0, -leg.upper_leg.z)
            
            x, y, z = temp_pos.get_position()
            self.debug_print(f"Transformed position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            # Reachability check
            target_to_foot = np.sqrt(x**2 + z**2)
            max_reach = abs(l1) + abs(l2)
            self.debug_print(f"Target distance: {target_to_foot:.3f}, Max reach: {max_reach:.3f}")
            
            if target_to_foot >= max_reach:
                self.debug_print("Target position unreachable")
                return float('nan'), float('nan'), float('nan')
            
            # Calculate remaining joint angles
            cos_arg = self.safe_divide((z**2 + x**2 - l1**2 - l2**2), (2 * l1 * l2))
            if abs(cos_arg) > 1:
                self.debug_print(f"Invalid cos_arg: {cos_arg}")
                return float('nan'), float('nan'), float('nan')
            
            lower_leg_joint = leg.knee_direction * self.safe_arccos(cos_arg)
            self.debug_print(f"lower_leg_joint: {np.degrees(lower_leg_joint):.2f}°")
            
            sin_term = l2 * np.sin(lower_leg_joint)
            cos_term = l1 + (l2 * np.cos(lower_leg_joint))
            
            if abs(cos_term) < self.EPSILON:
                self.debug_print("Invalid cos_term near zero")
                return float('nan'), float('nan'), float('nan')
            
            upper_leg_joint = (np.arctan2(x, z) - np.arctan2(sin_term, cos_term))
            upper_leg_joint += ik_alpha
            lower_leg_joint += ik_beta - ik_alpha
            
            if leg.knee_direction < 0 and upper_leg_joint < 0:
                upper_leg_joint += np.pi
            elif leg.knee_direction > 0 and upper_leg_joint > 0:
                upper_leg_joint += np.pi
            
            self.debug_print(f"Final angles: hip={np.degrees(hip_joint):.2f}°, "
                           f"upper={np.degrees(upper_leg_joint):.2f}°, "
                           f"lower={np.degrees(lower_leg_joint):.2f}°")
            
            return hip_joint, upper_leg_joint, lower_leg_joint
            
        except Exception as e:
            self.debug_print(f"Error in inverse kinematics calculation: {e}")
            return float('nan'), float('nan'), float('nan')

    def inverse_kinematics(self, foot_positions: List[Transformation]) -> List[float]:
        joint_positions = []
        
        for i, (leg, foot_pos) in enumerate(zip(self.legs, foot_positions)):
            self.debug_print(f"\nProcessing leg {i}")
            hip, upper, lower = self.inverse_kinematics_single_leg(leg, foot_pos)
            
            if np.isnan(hip) or np.isnan(upper) or np.isnan(lower):
                self.debug_print("Invalid calculation detected, returning all NaN")
                return [float('nan')] * 12
                
            joint_positions.extend([hip, upper, lower])
            
        return joint_positions