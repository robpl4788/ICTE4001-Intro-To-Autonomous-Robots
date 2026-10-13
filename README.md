# To start ros in wsl:

source /opt/ros/humble/setup.bash

source install/local_setup.bash


# To run local simulation:
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=office_world


# Faster Sim
ros2 launch diff_drive_robot_gz_sim ICTE4001_MRobot.launch.py world:=office_world 
ros2 launch turtlebot4_viz view_robot.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# To run GMap
ros2 run gmapper gmap --ros-args -p use_sim_time:=true