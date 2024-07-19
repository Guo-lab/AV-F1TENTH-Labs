source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build
ros2 run mpc mpc_node