# To start ros in wsl:

source /opt/ros/humble/setup.bash


# To Build project

colcon build

source install/local_setup.bash


# To run GMap

## Launch Sim
ros2 launch diff_drive_robot_gz_sim ICTE4001_MRobot.launch.py world:=office_world 

## Launch GMap and RViz2
### Robot will navigate to Nav2 Goal given in RViz2
### Robot will teleport as if kidnapped to 2D Pose Estimate given in RViz2
ros2 launch gmapper gmap.launch.py


# Known issues
Nav2 can get stuck on tight turns due to incorrect odometry.

Some warnings while building GMapping, but can be ignored