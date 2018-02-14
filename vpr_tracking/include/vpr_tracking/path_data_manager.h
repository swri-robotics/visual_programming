/**
 * @file path_data_manager.h
 * @copyright (c) 2018 Southwest Research Institute
 * @author  Jorge Nicho
 *  Created on: Jul 30, 2018
 */

#ifndef INCLUDE_VPR_TRACKING_PATH_DATA_MANAGER_H_
#define INCLUDE_VPR_TRACKING_PATH_DATA_MANAGER_H_

#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <visualization_msgs/MarkerArray.h>
#include <vpr_msgs/TrajectoryWaypoint.h>
#include <vpr_msgs/TrajectorySegment.h>
#include <vpr_msgs/EditPathWaypoints.h>
#include <vpr_tracking/snap_features.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace vpr_tracking
{

struct JumpThreshold
{
  double revolute = 0.0;
  double prismatic = 0.0;
};

struct EefStep
{
  double translation = 0.0;
  double rotation = 0.0;
};

enum RecordingModes: int
{
  WAYPOINT = 1,
  CONTINUOUS = 2,
  CONTINUOUS_TIME = 3
};

struct TrajectorySegmentInfo
{
  vpr_msgs::TrajectorySegment segment;                          /** The waypoints */
  RecordingModes recording_mode = RecordingModes::WAYPOINT;     /** The mode used to record the waypoints */
};

enum StateValidity: int
{
  OK = 1,
  INFEASIBLE ,
  DISTANT_CONFIGURATION
};

class PathDataManager
{
public:
  PathDataManager(ros::NodeHandle &nh, ros::NodeHandle &nhp);
  virtual ~PathDataManager();

  bool moveHome();
  bool addWaypoint();
  bool removeLastWaypoint();
  bool clearAllWaypoints();
  bool changeSnapMode();
  void startPoseTracking();
  void stopPoseTracking();
  bool startContinuousSegment();
  bool stopContinuousSegment();
  bool cancelContinuousSegment();
  bool startPathPreview(const moveit_msgs::DisplayTrajectory& disp_traj);
  void stopPathPreview();
  bool executePath(const moveit_msgs::DisplayTrajectory& disp_traj);
  boost::optional<moveit_msgs::DisplayTrajectory> planRobotTrajectories();

protected:

  void loadParams(ros::NodeHandle &nhp);
  void msgPublishTimerCallback(const ros::TimerEvent& e);
  void toolPoseCallback(const geometry_msgs::PoseStamped &pose);
  void jointStateCallback(const sensor_msgs::JointState& msg);
  void coloringState(const StateValidity &state_validity);
  boost::optional<vpr_msgs::TrajectoryWaypoint> getLatestWaypoint() const;
  void updatePathWaypointMarkers();
  jsk_rviz_plugins::OverlayText createInfoOverlay() const;
  bool withinDistance(const moveit_msgs::RobotState& st1_msg, const moveit_msgs::RobotState& st2_msg,
                                       const JumpThreshold& max_distance);
  boost::optional<geometry_msgs::PoseStamped> transformPose(const geometry_msgs::PoseStamped& pose_st,
                                                                             std::string to_frame);
  moveit_msgs::RobotStatePtr getCurrentState(ros::Duration max_delay = ros::Duration(0.2));
  void setDispRobotStateMsg(const moveit_msgs::DisplayRobotState& st);
  moveit_msgs::DisplayRobotState getDispRobotStateMsg();
  moveit_msgs::MoveItErrorCodes computeIK(const geometry_msgs::PoseStamped& pose, const moveit_msgs::RobotState& seed,
                                          moveit_msgs::RobotState& solution);
  moveit_msgs::MoveItErrorCodes computeFK(const sensor_msgs::JointState &joint_state, geometry_msgs::PoseStamped &pose);


  ros::NodeHandle nh_;
  ros::Subscriber tool_pose_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher current_path_marker_pub_;
  ros::Publisher info_overlay_pub_;
  ros::Publisher robot_state_ik_pub_;
  ros::Publisher disp_trajectory_pub_;
  ros::Publisher markers_pub_;
  ros::ServiceClient ik_srv_client_;
  ros::ServiceClient fk_srv_client_;
  ros::ServiceClient plan_trajectories_client_;
  ros::ServiceClient get_planning_scene_client_; /** @brief Used to get the latest robot state*/
  ros::Timer messages_publish_timer_;
  ros::Timer info_overlay_publish_timer_;
  ros::Timer waypoint_segment_building_timer_;

  // moveit
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveit_interface_;
  std::string move_group_name_;
  std::string home_pose_name_;	  /** @brief name of pre-recorded home pose */

  // waypoint management
  std::string trajectory_frame_;  /** @brief stores all trajectory waypoints in this frame */
  geometry_msgs::Pose prev_pose_;
  std::vector<TrajectorySegmentInfo> traj_segments_buffer_;
  visualization_msgs::MarkerArray waypoints_markers_;
  mutable std::mutex candidate_waypoint_mutex_;
  boost::optional<vpr_msgs::TrajectoryWaypoint> candidate_waypoint_ = boost::none;
  moveit_msgs::RobotState latest_valid_state_;
  EefStep robot_pose_eef_step_;   /** @brief When the new pose has moved farther than the step then the robot's pose will be
                                             computed */
  EefStep continuous_wp_eef_step_; /** @brief When the new pose has moved farther than the step then a new candidate waypoint
                                             will be saved during continuous waypoint recording */

  //snapping/post-processing
  SnapType current_snap_mode_ = SnapType::None;
  std::string features_frame_;
  std::vector<std::unique_ptr<SnapableFeature>> snap_features_;
  visualization_msgs::MarkerArray feature_markers_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // trajectory generation parameters
  JumpThreshold traj_jump_threshold_;
  EefStep traj_eef_step_;
  double max_vel_scaling_factor_ = 0.6;
  double max_acc_scaling_factor_ = 0.6;

  bool use_tool_pose_from_joint_states_ = false;
  std::string path_data_dir_;

  // robot state
  moveit::core::RobotStatePtr current_robot_st_;
  sensor_msgs::JointState current_joint_state_;
  mutable std::mutex disp_robot_st_mutex_;
  moveit_msgs::DisplayRobotStatePtr disp_robot_st_;
  std::atomic<bool> preview_traj_proceed_;
};

} /* namespace vpr_tracking */

#endif /* INCLUDE_VPR_TRACKING_PATH_DATA_MANAGER_H_ */
