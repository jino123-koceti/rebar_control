# Overall robot status message

string control_mode             # Current control mode: "idle", "manual", "auto", "navigating", "tying", "e_stop"
float32 battery_voltage         # Battery voltage (V)
bool gripper_ready              # Gripper is ready for operation
bool navigation_ready           # Navigation system is ready
geometry_msgs/Pose2D current_pose  # Current robot pose (x, y, theta)
