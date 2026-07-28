
# Run Button Listener Node
ros2 run tb4_cpp_prac1 tb4_cpp_node

# Run the action server
ros2 run tb4_cpp_prac1 tb4_cpp_action_server

# Undock action
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}


# Drive action
ros2 action send_goal /drive_distance_prac1 irobot_create_msgs/action/DriveDistance "{distance:0.5, max_translation_speed: 0.1}"
