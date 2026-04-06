# ur5e_hande_moveit_planner
plan using moveit with UR5e with hand E gripper attached

# install the driver
sudo apt-get install ros-humble-ur

# build source and run the planner
colcon build
source ~/.{ws}/install/setup.bash
ros2 launch urhe_config_new urhe_planner.launch.py

please note that the ip address for this it set to 192.168.1.100 on urhe_config_new/launch/urhe_planner.launch.py configure as your UR5e IP