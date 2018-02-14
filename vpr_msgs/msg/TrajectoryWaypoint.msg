# The tool pose the robot will try to reach; may get altered from original
geometry_msgs/PoseStamped tool_pose
# The original recorded tool pose, for logging purposes it should never be changed
geometry_msgs/PoseStamped raw_tool_pose
moveit_msgs/RobotState robot_state
duration time_from_start
int32 snap_mode
