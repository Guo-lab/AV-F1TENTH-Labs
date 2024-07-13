# ROS2 installation: https://github.com/f1tenth/f1tenth_gym_ros

# At AV-LAB:
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .

cd .. # back to AV-LAB
mkdir -p sim_ws/src
cd sim_ws/src

git clone https://github.com/f1tenth/f1tenth_gym_ros
cd f1tenth_gym_ros/config

# Change the following line in the file f1tenth_gym_ros/config/sim.yaml
# from
# map_file: "im_ws/src/f1tenth_gym/maps/levine.yaml"
# to
# map_path: '/home/gsq/AV-LAB/sim_ws/src/f1tenth_gym_ros/maps/levine'

cd ../../..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Run the simulator

# in one terminal
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# in another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard