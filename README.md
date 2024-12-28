# Quadruped Robot Controller with Inverse Kinematics

## Overview
This repository contains the implementation of a quadruped robot controller written in C++ and Python. The controller leverages Inverse Kinematics (IK) to calculate the joint angles required for the robot's legs to achieve specific foot positions in space. It also includes a ROS2 node setup to subscribe to foot position commands and publish corresponding joint trajectories.

The repository is split into two main components:

1. **C++ Controller Code**: Handles real-time calculations for foot positions, joint positions, and robot stability.
2. **Python Publisher**: Simulates user commands by publishing desired foot positions to the controller.

## Inverse Kinematics (IK)
Inverse Kinematics is a mathematical process used to determine the joint angles required for the end-effector (foot in this case) of a robotic manipulator to reach a desired position in 3D space. For a quadruped robot:

- Each leg is treated as a kinematic chain with three revolute joints (shoulder, knee, and ankle).
- IK calculations are performed individually for each leg based on the desired foot positions.

### IK Inputs:
1. **Foot Target Position**: Desired position of the foot in Cartesian coordinates \([x, y, z]\).
2. **Leg Parameters**: Lengths of the shoulder-to-knee and knee-to-ankle segments.

### IK Outputs:
1. Joint Angles \(\theta_1, \theta_2, \theta_3\): Angles for the shoulder, knee, and ankle joints, respectively.

### Equations:
Given the leg's link lengths \(l_1\) and \(l_2\), and the target foot position \([x, y, z]\):

1. Compute the distance from the shoulder to the target foot position:
   \[ r = \sqrt{x^2 + y^2 + z^2} \]

2. Compute the knee angle (\(\theta_2\)) using the law of cosines:
   \[ \theta_2 = \cos^{-1}\left(\frac{l_1^2 + l_2^2 - r^2}{2 \cdot l_1 \cdot l_2}\right) \]

3. Compute the shoulder angle (\(\theta_1\)) and ankle angle (\(\theta_3\)) using trigonometric relations and geometric projections.

### Implementation:
The IK is implemented in the `controlLoop_()` function in C++, which:
- Computes the desired foot positions based on velocity and pose commands.
- Uses IK to calculate the joint angles required to achieve the foot positions.
- Publishes joint commands to actuate the robot's legs.

## Robot Position Control
Position control allows precise movement and stabilization of the robot in 3D space. It involves:

1. **Body Pose Control**: Adjusting the robot's body orientation (roll, pitch, yaw) and position \([x, y, z]\).
2. **Foot Position Control**: Ensuring each foot is placed accurately to maintain stability and balance.

### Key Features:
- **Velocity Commands**: Subscribed via the `/cmd_vel` topic to determine the robot's linear and angular velocities.
- **Pose Commands**: Subscribed via the `/body_pose` topic to control the robot's body position and orientation.
- **Foot Position Commands**: Subscribed via the `/foot_positions` topic (newly added) to directly control foot placements.

## ROS2 Node Architecture

### C++ Controller Node:
**File:** `quadruped_controller.cpp`

#### Topics:
- **Subscribers:**
  - `/cmd_vel`: Accepts velocity commands.
  - `/body_pose`: Accepts desired body poses.
  - `/foot_positions`: Accepts desired foot positions.
- **Publishers:**
  - `/joint_states`: Publishes the calculated joint states.
  - `/joint_group_position_controller/command`: Publishes joint trajectory commands.
  - `/foot_contacts`: Publishes foot contact information for stability.

#### Main Components:
1. **IK Calculation**:
   - Computes target joint angles for the given foot positions.
2. **Control Loop**:
   - Periodically calculates joint positions and publishes commands.
3. **Foot Position Handling**:
   - Newly added functionality to accept specific foot positions.

### Python Publisher Node:
**File:** `test.py`

#### Purpose:
Simulates user commands by publishing desired foot positions to the `/foot_positions` topic.

#### Example Usage:
```python
import rclpy
from geometry_msgs.msg import Pose

def publish_foot_positions():
    node = rclpy.create_node('foot_position_publisher')
    publisher = node.create_publisher(Pose, '/foot_positions', 10)

    foot_positions = [
        [0.1, 0.1, -0.1],
        [-0.1, 0.1, -0.1],
        [-0.1, -0.1, -0.1],
        [0.1, -0.1, -0.1]
    ]

    for pos in foot_positions:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        publisher.publish(pose)
        node.get_logger().info(f'Published foot position: {pose}')

rclpy.init()
publish_foot_positions()
rclpy.shutdown()
```

## How to Run

### Prerequisites:
- ROS 2 Humble installed.
- C++ and Python development environment set up.

### Steps:
1. Build the C++ package:
   ```bash
   colcon build --packages-select quadruped_controller
   source install/setup.bash
   ```

2. Run the controller node:
   ```bash
   ros2 run quadruped_controller quadruped_controller_node
   ```

3. Run the Python publisher node:
   ```bash
   python3 test.py
   ```

## Future Work
- Add dynamic gait generation for rough terrains.
- Integrate real-time feedback for foot contacts.
- Expand support for different quadruped configurations.

## Contributions
Feel free to contribute to the project! Fork the repository, make changes, and create a pull request.

## License
This project is licensed under the BSD 3-Clause License. See the `LICENSE` file for details.

