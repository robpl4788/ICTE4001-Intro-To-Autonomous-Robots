# To start ros in wsl:

source /opt/ros/humble/setup.bash

source install/local_setup.bash


# To run local simulation:
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=office_world
