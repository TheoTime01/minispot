
ros2 launch quadruped_robot spot_bringup.launch.py
python3 test.py
ros2 run  joy joy_node
ros2 run champ_teleop joy_teleop.py


sudo chmod +0666 /dev/uinput
