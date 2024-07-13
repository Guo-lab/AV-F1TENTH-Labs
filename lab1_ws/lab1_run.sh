# FROM THE ROOT OF THE WORKSPACE
cd src
ros2 pkg create --build-type ament_cmake lab1_pkg

cd lab1_pkg/src
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp

##### rename ####

#### rosdep install ####
cd ../../
rosdep install -i --from-path src --rosdistro humble -y

#### In One terminal ####
colcon build --packages-select lab1_pkg
. install/setup.bash
ros2 run lab1_pkg talker


#### Another terminal ####
. install/setup.bash
ros2 run lab1_pkg relay

#### Or use ros2 launch  ####
ros2 launch lab1_pkg  lab1_launch.py

#### Check informtion ####
ros2 topic list
ros2 topic info /drive
ros2 topic echo drive
ros2 node list
ros2 node info /talker
ros2 node info /relay
