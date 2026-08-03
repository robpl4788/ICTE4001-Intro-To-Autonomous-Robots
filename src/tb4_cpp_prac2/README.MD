# Drive in an arc

Start the action server:
ros2 run tb4_cpp_prac2 tb4_arc_action


Run the arc action
ros2 action send_goal /drive_arc_prac2 irobot_create_msgs/action/DriveArc "{translate_direction: 1, angle: 1, radius: 0.3, max_translation_speed: 0.2}"