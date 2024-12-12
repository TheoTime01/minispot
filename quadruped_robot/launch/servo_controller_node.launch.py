import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file_name = 'urdf/spotmicroai_gen_ros.xacro'
    xacro_f = os.path.join(
        get_package_share_directory('quadruped_robot'),
        xacro_file_name)
    
    robot_desc = xacro.process_file(xacro_f).toxml() # il faut du coup aussi import xacro
    # conf_file_rviz = os.path.join(get_package_share_directory('quadruped_robot'),'config','medor.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='quadruped_robot',
            executable='leg_controller',
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', '/keyboard_input')  # Remap to custom topic
            ],
            arguments=['--ros-args', '--disable-timestamp']
        ),
                Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/tototime/ros2_ws/src/minispot/quadruped_robot/rviz/mini_spot.rviz']
        )
    ]
)