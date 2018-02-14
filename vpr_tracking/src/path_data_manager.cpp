/**
 * @file path_data_manager.cpp
 * @copyright (c) 2018 Southwest Research Institute
 * @author  Jorge Nicho
 *  Created on: Jul 30, 2018
 */

#include <cmath>
#include <fstream>
#include <functional>
#include <numeric>
#include <string>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <std_srvs/Trigger.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vpr_msgs/PlanTrajectories.h>
#include <vpr_tracking/path_data_manager.h>
#include <vpr_tracking/utils.h>
#include <yaml-cpp/yaml.h>

static const std::string PATH_DATA_DIR_PARAM = "path_data_dir";
static const std::string USE_TOOL_POSE_FROM_JOINT_STATES_PARAM = "use_tool_pose_from_joint_states";
static const std::string DISPLAY_TRAJECTORY_TOPIC = "trajectory_preview";
static const std::string PATH_LINE_MARKER_NS = "tool_path_line";
static const std::string PATH_DOTS_MARKER_NS = "tool_path_dots";
static const std::string INFO_OVERLAY_TOPIC = "info_overlay";
static const std::string CURRENT_PATH_TOPIC = "current_path";
static const std::string TOOL_POSE_TOPIC = "tool_pose";
static const std::string JOINT_STATE_TOPIC = "joint_states";
static const std::string ROBOT_STATE_IK_TOPIC = "robot_ik";
static const std::string MARKERS_TOPIC = "vpr_markers";
static const std::string IK_SERVICE = "compute_ik";
static const std::string FK_SERVICE = "compute_fk";
static const std::string PLAN_TRAJECTORIES_SERVICE = "plan_trajectories";
static const std::string GET_PLANNING_SCENE = "get_planning_scene";
static const double SEGMENT_BUILDER_TIMER_PERIOD = 0.1; // seconds
static const double MESSAGE_PUBLISH_TIMER_PERIOD = 0.01; // seconds
static const double TF_LOOKUP_PERIOD = 0.005;
static const double PATH_LINE_WIDTH = 0.01;
static const double PATH_DOT_SIZE = 0.02;
static const double IK_TIMEOUT = 0.005;
static const double SERVICE_WAIT_PERIOD = 5.0f;
static const int IK_ATTEMPTS = 2;
using RGBA = std::tuple<double,double,double,double>;

namespace robot_state_colors
{
  static const RGBA CONFIGURATION_CHANGED = std::make_tuple(0.7,0.7,0.1,1.0);
  static const RGBA INFEASIBLE = std::make_tuple(1.0,0.0,0.0,1.0);
}

static const RGBA RECORDING_MODE_WAYPOINT_LINE_RGBA = std::make_tuple(0.0,1.0,1.0,1.0);
static const RGBA RECORDING_MODE_WAYPOINT_DOT_RGBA = std::make_tuple(1.0,0.27,0.0,1.0);
static const RGBA RECORDING_MODE_CONTINUOUS_RGBA = std::make_tuple(0.8,0.8,0.2,1.0);
static const RGBA RECORDING_MODE_CONTINUOUS_TIME_RGBA = std::make_tuple(0.8,0.2,0.8,1.0);

enum ParamCodes: int
{
  MOVE_GROUP = 0,
  HOME_POSE_NAME,
  TRAJECTORY_FRAME,
  JUMP_THRESHOLD_REVOLUTE,
  JUMP_THRESHOLD_PRISMATIC,
  ROBOT_POSE_EEF_STEP_TRANSLATION,
  ROBOT_POSE_EEF_STEP_ROTATION,
  CONTINUOUS_WP_EEF_STEP_TRANSLATION,
  CONTINUOUS_WP_EEF_STEP_ROTATION,
  TRAJ_EEF_STEP_TRANSLATION,
  TRAJ_EEF_STEP_ROTATION,
  TRAJ_MAX_VEL_SCALING,
  TRAJ_MAX_ACC_SCALING
};
static const std::map<int,std::string> REQUIRED_PARAMS_MAP = {{MOVE_GROUP,"move_group"},
                                                              {HOME_POSE_NAME,"home_pose_name"},
                                                              {TRAJECTORY_FRAME,"trajectory_frame"}};
static const std::map<int,std::string> OPT_PARAMS_MAP = {{JUMP_THRESHOLD_REVOLUTE,"jump_threshold_revolute"},
                                                     {JUMP_THRESHOLD_PRISMATIC,"jump_threshold_prismatic"},
                                                     {ROBOT_POSE_EEF_STEP_TRANSLATION,"robot_pose_eef_step_translation"},
                                                     {ROBOT_POSE_EEF_STEP_ROTATION,"robot_pose_eef_step_rotation"},
                                                     {CONTINUOUS_WP_EEF_STEP_TRANSLATION,"continuous_wp_eef_step_translation"},
                                                     {CONTINUOUS_WP_EEF_STEP_ROTATION,"continuous_wp_eef_step_rotation"},
                                                     {TRAJ_EEF_STEP_TRANSLATION,"traj_eef_step_translation"},
                                                     {TRAJ_EEF_STEP_ROTATION,"traj_eef_step_rotation"},
                                                     {TRAJ_MAX_VEL_SCALING,"max_vel_scaling"},
                                                     {TRAJ_MAX_ACC_SCALING,"max_acc_scaling"}};

namespace vpr_tracking
{

bool equal(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2,
           double rotation_precision , double translation_precision)
{
  Eigen::Affine3d eigen_p1, eigen_p2;
  tf::poseMsgToEigen(p1,eigen_p1);
  tf::poseMsgToEigen(p2,eigen_p2);

  Eigen::Quaterniond q1(eigen_p1.rotation()), q2(eigen_p2.rotation());
  double rotation_distance = std::abs(q1.angularDistance(q2));
  double translation_distance = (eigen_p1.translation() - eigen_p2.translation()).norm();

  return (rotation_distance <= rotation_precision) && (translation_distance <= translation_precision);
}


RGBA getRecordingModeColor(int mode)
{
  static const std::map<int,RGBA> color_code_map = {{RecordingModes::WAYPOINT,RECORDING_MODE_WAYPOINT_LINE_RGBA},
                                       {RecordingModes::CONTINUOUS,RECORDING_MODE_CONTINUOUS_RGBA},
                                       {RecordingModes::CONTINUOUS_TIME,RECORDING_MODE_CONTINUOUS_TIME_RGBA}};
  if(color_code_map.count(mode) <= 0)
  {
    return std::make_tuple(0.0,0.0,0.0,1.0);
  }
  return color_code_map.at(mode);
}

boost::optional<moveit_msgs::RobotTrajectory> generateJointTrajectory(const moveit_msgs::RobotState& start_state_msg,
                                                           const moveit_msgs::RobotState& goal_state_msg,
                                          moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.setStartState(start_state_msg);
  move_group.setJointValueTarget(goal_state_msg.joint_state);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::MoveItErrorCodes error_code = move_group.plan(plan);
  return boost::make_optional(error_code.val == error_code.SUCCESS,plan.trajectory_);
}

visualization_msgs::MarkerArray createPathLineMarker(std::string frame_id, const std::vector<vpr_msgs::TrajectoryWaypoint>& waypoints,
                                                 const RGBA& color, int marker_id)
{
  using namespace visualization_msgs;
  MarkerArray markers;
  Marker line_marker;
  line_marker.type = Marker::LINE_STRIP;
  line_marker.action = Marker::ADD;
  line_marker.color.r = std::get<0>(color);
  line_marker.color.g = std::get<1>(color);
  line_marker.color.b = std::get<2>(color);
  line_marker.color.a = std::get<3>(color);
  line_marker.lifetime = ros::Duration(0.2);
  line_marker.ns = PATH_LINE_MARKER_NS;
  line_marker.header.frame_id = frame_id;
  line_marker.id = marker_id;
  line_marker.scale.x = line_marker.scale.y = line_marker.scale.z = PATH_LINE_WIDTH; //

  for(const auto& w : waypoints)
  {
    line_marker.points.push_back(w.tool_pose.pose.position);
  }

  markers.markers.push_back(line_marker);
  return std::move(markers);
}

visualization_msgs::MarkerArray createPathDotsMarker(std::string frame_id, const std::vector<vpr_msgs::TrajectoryWaypoint>& waypoints,
                                                 const RGBA& color, int marker_id)
{
  using namespace visualization_msgs;
  MarkerArray markers;
  Marker dots_marker;
  dots_marker.type = Marker::SPHERE_LIST;
  dots_marker.action = Marker::ADD;
  dots_marker.color.r = std::get<0>(color);
  dots_marker.color.g = std::get<1>(color);
  dots_marker.color.b = std::get<2>(color);
  dots_marker.color.a = std::get<3>(color);
  dots_marker.lifetime = ros::Duration(0.2);
  dots_marker.ns = PATH_DOTS_MARKER_NS;
  dots_marker.header.frame_id = frame_id;
  dots_marker.id = marker_id;
  dots_marker.scale.x = dots_marker.scale.y = dots_marker.scale.z = PATH_DOT_SIZE; //

  for(const auto& w : waypoints)
  {
    dots_marker.points.push_back(w.tool_pose.pose.position);
  }

  markers.markers.push_back(dots_marker);
  return std::move(markers);
}

std::string makeTimeStampString()
{
  auto curr_time_pt = std::chrono::system_clock::now();
  auto curr_time_t = std::chrono::system_clock::to_time_t(curr_time_pt);
  auto curr_localtime = std::localtime(&curr_time_t);
  auto curr_millis = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time_pt.time_since_epoch());
  int centisecs = int(curr_millis.count() % 1000) / 10;

  std::ostringstream ss;
  ss << std::put_time(curr_localtime, "%y%m%d-%H%M%S");

  std::string id = ss.str();
  return id;
}

void writePathData(const std::vector<vpr_msgs::TrajectorySegment>& traj_segments,
                   const moveit_msgs::DisplayTrajectory& planned_traj,
                   const geometry_msgs::TransformStamped& workpiece_transform,
                   const std::string &path_data_dir_)
{
  YAML::Node teach_data_node;
  for (const vpr_msgs::TrajectorySegment& segment : traj_segments)
  {
    YAML::Node segment_node;
    for(const vpr_msgs::TrajectoryWaypoint& waypoint : segment.waypoints)
    {
      YAML::Node wpt_node;
      wpt_node["tool_pose"]["position"]["x"] = waypoint.tool_pose.pose.position.x;
      wpt_node["tool_pose"]["position"]["y"] = waypoint.tool_pose.pose.position.y;
      wpt_node["tool_pose"]["position"]["z"] = waypoint.tool_pose.pose.position.z;
      wpt_node["tool_pose"]["orientation"]["w"] = waypoint.tool_pose.pose.orientation.w;
      wpt_node["tool_pose"]["orientation"]["x"] = waypoint.tool_pose.pose.orientation.x;
      wpt_node["tool_pose"]["orientation"]["y"] = waypoint.tool_pose.pose.orientation.y;
      wpt_node["tool_pose"]["orientation"]["z"] = waypoint.tool_pose.pose.orientation.z;

      wpt_node["raw_tool_pose"]["position"]["x"] = waypoint.raw_tool_pose.pose.position.x;
      wpt_node["raw_tool_pose"]["position"]["y"] = waypoint.raw_tool_pose.pose.position.y;
      wpt_node["raw_tool_pose"]["position"]["z"] = waypoint.raw_tool_pose.pose.position.z;
      wpt_node["raw_tool_pose"]["orientation"]["w"] = waypoint.raw_tool_pose.pose.orientation.w;
      wpt_node["raw_tool_pose"]["orientation"]["x"] = waypoint.raw_tool_pose.pose.orientation.x;
      wpt_node["raw_tool_pose"]["orientation"]["y"] = waypoint.raw_tool_pose.pose.orientation.y;
      wpt_node["raw_tool_pose"]["orientation"]["z"] = waypoint.raw_tool_pose.pose.orientation.z;

      wpt_node["snap_mode"] = waypoint.snap_mode;

      const sensor_msgs::JointState& joint_states = waypoint.robot_state.joint_state;
      for(int i=0; i < joint_states.name.size(); ++i)
      {
        wpt_node["joint_states"][joint_states.name[i]] = joint_states.position[i];
      }
      segment_node["waypoints"].push_back(wpt_node);
    }
    teach_data_node.push_back(segment_node);
  }

  YAML::Node plan_data_node;
  for (const moveit_msgs::RobotTrajectory& robot_traj : planned_traj.trajectory)
  {
    YAML::Node joint_traj_node;
    for (const auto& joint_name : robot_traj.joint_trajectory.joint_names)
    {
      joint_traj_node["joint_names"].push_back(joint_name);
    }
    for (const trajectory_msgs::JointTrajectoryPoint& traj_pt : robot_traj.joint_trajectory.points)
    {
      YAML::Node traj_pt_node;
      for (double position : traj_pt.positions)
      {
        traj_pt_node["positions"].push_back(position);
      }
      if (traj_pt.velocities.size() > 0)
      {
        for (double velocity : traj_pt.velocities)
        {
          traj_pt_node["velocities"].push_back(velocity);
        }
      }
      if (traj_pt.accelerations.size() > 0)
      {
        for (double accel : traj_pt.accelerations)
        {
          traj_pt_node["accelerations"].push_back(accel);
        }
      }
      if (traj_pt.effort.size() > 0)
      {
        for (double eff : traj_pt.effort)
        {
          traj_pt_node["effort"].push_back(eff);
        }
      }
      traj_pt_node["time_from_start"] = traj_pt.time_from_start.toSec();
      joint_traj_node["points"].push_back(traj_pt_node);
    }
    plan_data_node.push_back(joint_traj_node);
  }

  YAML::Node data_node;
  data_node["teach_path"] = teach_data_node;
  data_node["planned_path"] = plan_data_node;

  if (! workpiece_transform.child_frame_id.empty())
  {
    data_node["trajectory_frame"] = workpiece_transform.header.frame_id;
    data_node["workpiece_frame"] = workpiece_transform.child_frame_id;
    data_node["workpiece_transform"]["translation"]["x"] = workpiece_transform.transform.translation.x;
    data_node["workpiece_transform"]["translation"]["y"] = workpiece_transform.transform.translation.y;
    data_node["workpiece_transform"]["translation"]["z"] = workpiece_transform.transform.translation.z;
    data_node["workpiece_transform"]["rotation"]["w"] = workpiece_transform.transform.rotation.w;
    data_node["workpiece_transform"]["rotation"]["x"] = workpiece_transform.transform.rotation.x;
    data_node["workpiece_transform"]["rotation"]["y"] = workpiece_transform.transform.rotation.y;
    data_node["workpiece_transform"]["rotation"]["z"] = workpiece_transform.transform.rotation.z;
  }

  YAML::Emitter out;
  out << data_node;

  std::string filename = "vpr_" + makeTimeStampString() + ".yaml";
  boost::filesystem::path filepath = boost::filesystem::path(path_data_dir_) / filename;

  std::ofstream fout(filepath.string());
  if (! fout.good())
  {
    ROS_ERROR_STREAM("Failed to open file for writing: " << filepath);
  }
  else
  {
    fout << out.c_str();
    fout.close();
  }
}

SnapType nextSnapMode(const SnapType &sm)
{
  switch (sm)
  {
    case SnapType::None:
      return SnapType::Point;
    case SnapType::Point:
      return SnapType::Line;
    case SnapType::Line:
      return SnapType::Face;
    case SnapType::Face:
      return SnapType::None;
  }
}

boost::optional<geometry_msgs::PoseStamped> snapWaypoint(
    const geometry_msgs::PoseStamped& waypoint,
    const std::vector<std::unique_ptr<SnapableFeature>>& features,
    const std::string& features_frame,
    SnapType snap_type,
    const tf2_ros::Buffer &tf_buffer)
{
  //Find which features match
  std::vector<SnapableFeature*> selected_features;
  for (auto& feature : features)
  {
    if (feature->type() == snap_type)
    {
      selected_features.push_back(feature.get());
    }
  }

  if (selected_features.empty())
  {
    ROS_WARN("No relevant snap features");
    return boost::none;
  }

  //Transform robot waypoint to feature frame (use current time)
  geometry_msgs::PoseStamped robot_pose = waypoint;
  robot_pose.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped feature_pose = tf_buffer.transform(robot_pose, features_frame, ros::Duration(1.0));

  //Do snapping (checking for multiple snaps)
  Eigen::Vector3d xyz{
    feature_pose.pose.position.x,
    feature_pose.pose.position.y,
    feature_pose.pose.position.z,
  };
  boost::optional<Eigen::Vector3d> snapped_point = boost::none;
  int num_snaps = 0;
  for (auto& snap_feature : selected_features)
  {
    boost::optional<Eigen::Vector3d> snapped = snap_feature->snap(xyz);
    if (snapped)
    {
      num_snaps++;
      snapped_point = snapped;
    }
  }
  if (num_snaps > 1)
  {
    ROS_WARN("Multiple features close enough to snap for point at (%f,%f,%f), snap type: '%s', no snap performed",
        xyz.x(), xyz.y(), xyz.z(), to_string(snap_type).c_str());
    return boost::none;
  }
  if (! snapped_point) //No feature close enough
  {
    return boost::none;
  }

  //Transform back to planning frame
  geometry_msgs::PoseStamped snapped_pose;
  snapped_pose.header = feature_pose.header;
  snapped_pose.pose.position.x = snapped_point->x();
  snapped_pose.pose.position.y = snapped_point->y();
  snapped_pose.pose.position.z = snapped_point->z();
  snapped_pose.pose.orientation = feature_pose.pose.orientation;
  geometry_msgs::PoseStamped snapped_robot_pose = tf_buffer.transform(
      snapped_pose, waypoint.header.frame_id, ros::Duration(1.0));

  return snapped_robot_pose;
}

PathDataManager::PathDataManager(ros::NodeHandle &nh, ros::NodeHandle &nhp) :
    nh_(nh),
    tf_listener_(tf_buffer_)
{
  loadParams(nhp);

  // initializing trajectory buffer
  traj_segments_buffer_.reserve(20);

  // create moveit interface
  moveit_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      move_group_name_);
  moveit_interface_->startStateMonitor();

  // subscribers, publishers and clients
  info_overlay_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>(INFO_OVERLAY_TOPIC,10);
  current_path_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(CURRENT_PATH_TOPIC,10);
  robot_state_ik_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>(ROBOT_STATE_IK_TOPIC, 10);
  disp_trajectory_pub_ = nh.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_TRAJECTORY_TOPIC,10);
  markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(MARKERS_TOPIC,10);
  ik_srv_client_ = nh.serviceClient<moveit_msgs::GetPositionIK>(IK_SERVICE);
  fk_srv_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>(FK_SERVICE);
  plan_trajectories_client_ = nh.serviceClient<vpr_msgs::PlanTrajectories>(PLAN_TRAJECTORIES_SERVICE);
  get_planning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE);

  if(use_tool_pose_from_joint_states_)
  {
    // computes the tool pose from ik
    joint_state_sub_ = nh.subscribe(JOINT_STATE_TOPIC, 1, &PathDataManager::jointStateCallback, this);
  }
  else
  {
    // gets the tool pose from a ROS message
    tool_pose_sub_ = nh.subscribe(TOOL_POSE_TOPIC, 1,&PathDataManager::toolPoseCallback, this);
  }

  // wait for service clients
  std::vector<ros::ServiceClient*> srv_clients = {&ik_srv_client_, &plan_trajectories_client_, &get_planning_scene_client_};
  if(!std::all_of(srv_clients.begin(),srv_clients.end(),[](ros::ServiceClient* client) -> bool{
    if(!client->waitForExistence(ros::Duration(SERVICE_WAIT_PERIOD)))
    {
      ROS_ERROR("ROS service %s was not found",client->getService().c_str());
      return false;
    }
    return true;
  }))
  {
    throw std::runtime_error("One or more ros services were not found");
  }
  ROS_INFO("All service for node %s were found",ros::this_node::getName().c_str());

  // initializing robot state data
  current_robot_st_.reset(new moveit::core::RobotState(moveit_interface_->getRobotModel()));
  current_robot_st_->setToDefaultValues();
  getCurrentState(); // updates current_robot_st_
  robot_state::robotStateToRobotStateMsg(*current_robot_st_,latest_valid_state_);
  disp_robot_st_.reset(new moveit_msgs::DisplayRobotState());
  disp_robot_st_->state = latest_valid_state_;

  // timers
  messages_publish_timer_ = nh.createTimer(ros::Duration(MESSAGE_PUBLISH_TIMER_PERIOD),
                                              &PathDataManager::msgPublishTimerCallback, this);

  auto send_info_overlay = [this](const ros::TimerEvent &ev)
  {
    info_overlay_pub_.publish(createInfoOverlay());
  };
  info_overlay_publish_timer_ = nh.createTimer(ros::Duration(MESSAGE_PUBLISH_TIMER_PERIOD), send_info_overlay);

  ROS_DEBUG("MoveIt! interface ready to plan with planning frame '%s'",moveit_interface_->getPlanningFrame().c_str());
}

PathDataManager::~PathDataManager()
{

}

bool PathDataManager::moveHome()
{
	if(!moveit_interface_->setNamedTarget(home_pose_name_))
	{
		return false;
	}

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::planning_interface::MoveItErrorCode error_code = moveit_interface_->plan(plan);
	if(!error_code)
	{
		ROS_ERROR("Failed to plan to home pose '%s'",home_pose_name_.c_str());
		return false;
	}

	error_code = moveit_interface_->execute(plan);
	if(!error_code)
	{
		ROS_ERROR("Failed execution of trajectory to home pose");
		return false;
	}

	return true;
}

void PathDataManager::loadParams(ros::NodeHandle &nhp)
{
  // required parameters
  bool required_success = nhp.getParam(REQUIRED_PARAMS_MAP.at(MOVE_GROUP), move_group_name_) &&
      nhp.getParam(REQUIRED_PARAMS_MAP.at(HOME_POSE_NAME), home_pose_name_) &&
      nhp.getParam(REQUIRED_PARAMS_MAP.at(TRAJECTORY_FRAME), trajectory_frame_);

  std::string required_list_str = std::accumulate(REQUIRED_PARAMS_MAP.begin(),REQUIRED_PARAMS_MAP.end(),
                                              std::string(""),[](std::string ss,const std::map<int,std::string>::value_type& v){
    return ss + " " + v.second;
  });

  if (! required_success)
  {
    throw std::runtime_error("One or more required parameters were not found: " + required_list_str);
  }

  // optional parameters
  bool optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(JUMP_THRESHOLD_REVOLUTE),traj_jump_threshold_.revolute,M_PI/3);
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(JUMP_THRESHOLD_PRISMATIC),
                                       traj_jump_threshold_.prismatic,0.05) && optional_success;

  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(ROBOT_POSE_EEF_STEP_TRANSLATION),
                                       robot_pose_eef_step_.translation,0.02) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(ROBOT_POSE_EEF_STEP_ROTATION),
                                       robot_pose_eef_step_.rotation,M_PI/20.0) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(CONTINUOUS_WP_EEF_STEP_TRANSLATION),
                                       continuous_wp_eef_step_.translation,0.02) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(CONTINUOUS_WP_EEF_STEP_ROTATION),
                                       continuous_wp_eef_step_.rotation,M_PI/20.0) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(TRAJ_EEF_STEP_TRANSLATION),
                                       traj_eef_step_.translation,0.02) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(TRAJ_EEF_STEP_ROTATION),
                                       traj_eef_step_.rotation,M_PI/40.0) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(TRAJ_MAX_VEL_SCALING),
                                       max_vel_scaling_factor_,max_vel_scaling_factor_) && optional_success;
  optional_success = nhp.param<double>(OPT_PARAMS_MAP.at(TRAJ_MAX_ACC_SCALING),
                                       max_acc_scaling_factor_,max_acc_scaling_factor_) && optional_success;

  std::string opt_list_str = std::accumulate(OPT_PARAMS_MAP.begin(),OPT_PARAMS_MAP.end(),
                                              std::string(""),[](std::string ss,const std::map<int,std::string>::value_type& v){
    return ss + " " + v.second;
  });

  ROS_WARN_COND(!optional_success,"One or more optional parameters were set to default values %s",opt_list_str.c_str());

  //use_tool_pose_from_joint_states_ = true;
  if (! nhp.getParam(USE_TOOL_POSE_FROM_JOINT_STATES_PARAM, use_tool_pose_from_joint_states_))
    throw std::runtime_error("Required parameter '" + USE_TOOL_POSE_FROM_JOINT_STATES_PARAM + "' not found");

  if (! nhp.getParam(PATH_DATA_DIR_PARAM, path_data_dir_))
    throw std::runtime_error("Required parameter '" + PATH_DATA_DIR_PARAM + "' not found");

  if (! nhp.getParam("snap/frame", features_frame_))
    throw std::runtime_error("Required parameter '~snap/frame' not found");

  XmlRpc::XmlRpcValue features_param;
  if (! nhp.getParam("snap/features", features_param))
    throw std::runtime_error("Required parameter '~snap/features' not found");

  if (features_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error("Parameter '~snap/features' not an array");

  try
  {
    ROS_INFO("Loading %i snap features", features_param.size());
    for (int i=0; i < features_param.size(); ++i)
    {
      snap_features_.push_back(snapFeatureFromParam(features_param[i]));
    }
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    throw std::runtime_error("Problem loading '~snap/features' parameter: " + ex.getMessage());
  }
}

void PathDataManager::msgPublishTimerCallback(const ros::TimerEvent& e)
{
  // publish path
  if(!waypoints_markers_.markers.empty())
  {
    current_path_marker_pub_.publish(waypoints_markers_);
  }

  // publish display state
  robot_state_ik_pub_.publish(getDispRobotStateMsg());
}

bool PathDataManager::addWaypoint()
{
  if(traj_segments_buffer_.empty())
  {
    TrajectorySegmentInfo ti = {.segment = vpr_msgs::TrajectorySegment(),.recording_mode = RecordingModes::WAYPOINT};
    traj_segments_buffer_.push_back(ti);
  }
  else
  {
    if(traj_segments_buffer_.back().recording_mode != RecordingModes::WAYPOINT)
    {
      TrajectorySegmentInfo ti = {.segment = vpr_msgs::TrajectorySegment(),.recording_mode = RecordingModes::WAYPOINT};
      traj_segments_buffer_.push_back(ti);
    }
  }

  auto get_waypoint_threadsafe = [this]() -> boost::optional<vpr_msgs::TrajectoryWaypoint>
  {
    std::lock_guard<std::mutex> lock{candidate_waypoint_mutex_};
    return candidate_waypoint_;
  };

  auto candidate_wpt = get_waypoint_threadsafe();

  if (! candidate_wpt.is_initialized())
  {
    ROS_ERROR_STREAM("Current waypoint is not feasible");
    return false;
  }

  if (current_snap_mode_ != SnapType::None)
  {
    try
    {
      boost::optional<geometry_msgs::PoseStamped> snapped_pose = snapWaypoint(
          candidate_wpt->tool_pose, snap_features_, features_frame_, current_snap_mode_, tf_buffer_);

      if (snapped_pose) //recompute IK
      {
        moveit_msgs::RobotState ik_solution;
        moveit_msgs::MoveItErrorCodes error_code = computeIK(*snapped_pose, candidate_wpt->robot_state, ik_solution);
        if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_ERROR("Failed to adjust IK result of snapped point");
        }
        else //adjust the tool pose
        {
          candidate_wpt->robot_state = ik_solution;
          candidate_wpt->tool_pose = *snapped_pose;
        }
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("TF error when attempting to snap: " << ex.what());
    }
  }

  std::vector<vpr_msgs::TrajectoryWaypoint>& waypoints_buffer = traj_segments_buffer_.back().segment.waypoints;
  waypoints_buffer.push_back(*candidate_wpt);

  // updating markers
  updatePathWaypointMarkers();

  return true;
}

bool PathDataManager::removeLastWaypoint()
{
  if(traj_segments_buffer_.empty())
  {
    ROS_WARN("No waypoints left to delete");
    return true;
  }

  // removing last point
  if(!traj_segments_buffer_.back().segment.waypoints.empty())
  {
    traj_segments_buffer_.back().segment.waypoints.pop_back();
  }

  // removing empty segments
  if(traj_segments_buffer_.back().segment.waypoints.empty())
  {
    traj_segments_buffer_.pop_back();
  }

  // updating markers
  updatePathWaypointMarkers();
  return true;
}

bool PathDataManager::clearAllWaypoints()
{
  traj_segments_buffer_.clear();

  // updating markers
  waypoints_markers_ = createPathLineMarker("",{},
                                        getRecordingModeColor(-1),1);

  return traj_segments_buffer_.empty();
}

bool PathDataManager::changeSnapMode()
{
  SnapType next_mode = nextSnapMode(current_snap_mode_);
  ROS_INFO_STREAM("Changing snap mode from " << to_string(current_snap_mode_) << " to " << to_string(next_mode));
  current_snap_mode_ = next_mode;

  std::vector<SnapableFeature*> current_features;
  for (auto& feature : snap_features_)
  {
    if (feature->type() == current_snap_mode_)
      current_features.push_back(feature.get());
  }

  //clear current markers
  for (auto& marker : feature_markers_.markers)
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  markers_pub_.publish(feature_markers_);
  feature_markers_.markers.clear();

  //get new markers
  ros::Time t{ros::Time::now()};
  for (auto& feature : current_features)
  {
    visualization_msgs::Marker marker = feature->visualization();
    marker.header.stamp = t;
    marker.header.frame_id = features_frame_;
    feature_markers_.markers.push_back(std::move(marker));
  }
  markers_pub_.publish(feature_markers_);
  return true;
}

bool PathDataManager::startPathPreview(const moveit_msgs::DisplayTrajectory& disp_traj)
{
  disp_trajectory_pub_.publish(disp_traj);
  ROS_INFO_STREAM("Started Trajectory Preview");
  double traj_time = std::accumulate(disp_traj.trajectory.begin(),disp_traj.trajectory.end(),0.0,
                                     [](double val, const moveit_msgs::RobotTrajectory& traj){
    val += traj.joint_trajectory.points.back().time_from_start.toSec();
    return val;
  });

  ros::Time start_time = ros::Time::now();
  ros::Duration pause_duration = ros::Duration(0.1);
  ros::Duration time_elapsed = ros::Duration(0.0);
  preview_traj_proceed_ = true;
  while(time_elapsed.toSec() <= traj_time && preview_traj_proceed_)
  {
    pause_duration.sleep();
    time_elapsed += pause_duration;
  }

  bool completed = time_elapsed.toSec() >= traj_time;
  ROS_INFO_COND(completed,"Finished Trajectory Preview");
  ROS_INFO_COND(!completed,"Interrupted Trajectory Preview");
  return true;
}

void PathDataManager::stopPathPreview()
{
  preview_traj_proceed_ = false;
  moveit_msgs::DisplayTrajectory disp_traj;
  disp_traj.model_id = moveit_interface_->getRobotModel()->getName();

  // create null trajectory
  trajectory_msgs::JointTrajectory jt;
  jt.joint_names = moveit_interface_->getActiveJoints();
  jt.points.resize(1);

  std::transform(jt.joint_names.begin(),jt.joint_names.end(),
                 std::back_inserter(jt.points.front().positions),[&](std::string jn){
    return current_robot_st_ ? current_robot_st_->getVariablePosition(jn) : 0.0;
  });
  jt.points.front().time_from_start = ros::Duration(0.01);
  disp_traj.trajectory.resize(1);
  disp_traj.trajectory.front().joint_trajectory = jt;

  disp_trajectory_pub_.publish(disp_traj);
}

bool PathDataManager::executePath(const moveit_msgs::DisplayTrajectory& disp_traj)
{
  using MoveIt = moveit::planning_interface::MoveGroupInterface;

  // attempting to grab current robot state
  moveit::core::RobotStatePtr robot_st = moveit_interface_->getCurrentState();
  if(!robot_st)
  {
    robot_st.reset(new moveit::core::RobotState(moveit_interface_->getRobotModel()));
    robot_st->setToDefaultValues();
  }

  // executing each trajectory
  int counter = 1;
  for(const auto& traj: disp_traj.trajectory)
  {
    // creating start state
    moveit_msgs::RobotState st_msg;
    const trajectory_msgs::JointTrajectoryPoint& jp = traj.joint_trajectory.points.front();
    const auto& jnames = traj.joint_trajectory.joint_names;
    for(std::size_t j = 0 ; j < jnames.size() ; j++)
    {
      const moveit::core::JointModel* jmodel = moveit_interface_->getRobotModel()->getJointModel(jnames[j]);
      double jval = jp.positions[j];
      robot_st->setJointPositions(jmodel,{jval});
    }

    robot_state::robotStateToRobotStateMsg(*robot_st,st_msg);
    MoveIt::Plan plan;
    plan.trajectory_ = traj;
    plan.start_state_ = st_msg;

    ROS_INFO("Executing trajectory %i", counter);
    moveit_msgs::MoveItErrorCodes error_code = moveit_interface_->execute(plan);
    if(error_code.val != error_code.SUCCESS)
    {
      ROS_ERROR("Failed to execute trajectory %i, error code: %i",counter,error_code.val);
      return false;
    }
    counter++;
  }
  ROS_INFO("Finished executing all %lu trajectories", disp_traj.trajectory.size());
  return true;
}

boost::optional<moveit_msgs::DisplayTrajectory> PathDataManager::planRobotTrajectories()
{
  std::string err_msg;
  if(traj_segments_buffer_.empty())
  {
    err_msg  = "No waypoints have been added";
    ROS_WARN_STREAM(err_msg );
    return boost::none;
  }

  // grab current robot state
  moveit_msgs::RobotStatePtr current_state_msg = getCurrentState();
  if(!current_state_msg)
  {
    return boost::none;
  }
  ROS_DEBUG_STREAM("Start Robot State Position:\n"<<current_state_msg->joint_state<<std::endl);

  // trajectory details
  vpr_msgs::PlanTrajectories srv;
  srv.request.group_name = move_group_name_;
  srv.request.start_state = *current_state_msg;
  srv.request.jump_threshold_revolute = traj_jump_threshold_.revolute;
  srv.request.jump_threshold_prismatic = traj_jump_threshold_.prismatic;
  srv.request.max_eef_step_translation = traj_eef_step_.translation;
  srv.request.max_eef_step_rotation = traj_eef_step_.rotation;
  srv.request.max_vel_scaling_factor = max_vel_scaling_factor_;
  srv.request.max_acc_scaling_factor = max_acc_scaling_factor_;

  // adding trajectory segments
  std::transform(traj_segments_buffer_.begin(),traj_segments_buffer_.end(),
                 std::back_inserter(srv.request.trajectory_segments),[](const TrajectorySegmentInfo& ti){
    return ti.segment;
  });


  if(!plan_trajectories_client_.call(srv))
  {
    err_msg = "Failed to call the planning service " + plan_trajectories_client_.getService();
    ROS_ERROR_STREAM(err_msg);
    return boost::none;
  }

  if(!srv.response.success)
  {
    err_msg = "Planning failed";
    ROS_ERROR_STREAM(err_msg);
    return boost::none;
  }

  //Store the location of the workpiece so the trajectory can be compared against the known feature locations
  //Not ideal to look this up here since it might have changed since recording, will need to rethink this
  geometry_msgs::TransformStamped workpiece_transform;
  try
  {
    workpiece_transform = tf_buffer_.lookupTransform(trajectory_frame_, features_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM("Failed to get current workpiece transform: " << ex.what());
  }

  writePathData(srv.request.trajectory_segments, srv.response.robot_trajectories, workpiece_transform, path_data_dir_);

  return srv.response.robot_trajectories;
}

bool PathDataManager::startContinuousSegment()
{
  TrajectorySegmentInfo ti = {.segment = vpr_msgs::TrajectorySegment(), .recording_mode = RecordingModes::CONTINUOUS};
  traj_segments_buffer_.push_back(ti);
  auto timer_cb = [&](const ros::TimerEvent& evnt)
  {
    std::lock_guard<std::mutex> lock{candidate_waypoint_mutex_};
    std::vector<vpr_msgs::TrajectoryWaypoint>& waypoints = traj_segments_buffer_.back().segment.waypoints;
    if(candidate_waypoint_.is_initialized())
    {

      boost::optional<vpr_msgs::TrajectoryWaypoint> latest_wp = getLatestWaypoint();
      if(latest_wp.is_initialized())
      {
        // compare to latest waypoint tool pose, return if candidate is too close
        if(equal(latest_wp->tool_pose.pose,candidate_waypoint_->tool_pose.pose,continuous_wp_eef_step_.rotation,
                 continuous_wp_eef_step_.translation))
        {
          return;
        }

        // check that robot configuration isn't too far
        if(withinDistance(candidate_waypoint_->robot_state,waypoints.back().robot_state,traj_jump_threshold_))
        {
          waypoints.push_back(*candidate_waypoint_);
        }
      }
      else
      {
        // no waypoints yet so just insert
        waypoints.push_back(*candidate_waypoint_);
      }
      candidate_waypoint_ = boost::none;

      // updating markers
      updatePathWaypointMarkers();
    }
  };

  waypoint_segment_building_timer_ = nh_.createTimer(ros::Duration(SEGMENT_BUILDER_TIMER_PERIOD),timer_cb);
  return true;
}

bool PathDataManager::stopContinuousSegment()
{
  waypoint_segment_building_timer_.stop();
  return true;
}

bool PathDataManager::cancelContinuousSegment()
{
  waypoint_segment_building_timer_.stop();
  traj_segments_buffer_.pop_back();
  updatePathWaypointMarkers();
  return true;
}

boost::optional<geometry_msgs::PoseStamped> PathDataManager::transformPose(const geometry_msgs::PoseStamped& pose_st,
                                                                           std::string to_frame)
{
  geometry_msgs::TransformStamped transform_st;
  geometry_msgs::PoseStamped transformed_pose_st;
  try
  {
    transform_st = tf_buffer_.lookupTransform(to_frame,pose_st.header.frame_id,ros::Time(0),ros::Duration(TF_LOOKUP_PERIOD));
  }
  catch(tf2::TransformException& e)
  {
    ROS_ERROR_STREAM(e.what());
    return boost::none;
  }

  Eigen::Affine3d src_to_target, target_pose,src_pose;
  tf::transformMsgToEigen(transform_st.transform,src_to_target);
  tf::poseMsgToEigen(pose_st.pose,src_pose);
  target_pose = src_to_target * src_pose;
  transformed_pose_st.header.frame_id = to_frame;
  tf::poseEigenToMsg(target_pose,transformed_pose_st.pose);

  return transformed_pose_st;
}

void PathDataManager::startPoseTracking()
{
  if(use_tool_pose_from_joint_states_)
  {
    // computes the tool pose from ik
    joint_state_sub_ = nh_.subscribe(JOINT_STATE_TOPIC, 1, &PathDataManager::jointStateCallback, this);
  }
  else
  {
    // gets the tool pose from a ROS message
    tool_pose_sub_ = nh_.subscribe(TOOL_POSE_TOPIC, 1,&PathDataManager::toolPoseCallback, this);
  }
}

void PathDataManager::stopPoseTracking()
{
  moveit_msgs::DisplayRobotState disp_msg;
  disp_msg.state = latest_valid_state_;
  const std::vector<std::string>& link_names = moveit_interface_->getLinkNames();
  for(const std::string& ln : link_names)
  {
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = ln;
    obj_color.color.a = 0.0;
    disp_msg.highlight_links.push_back(obj_color);
  }
  setDispRobotStateMsg(disp_msg);
  tool_pose_sub_.shutdown();
  joint_state_sub_.shutdown();
}

moveit_msgs::MoveItErrorCodes PathDataManager::computeIK(const geometry_msgs::PoseStamped& pose,
                                                         const moveit_msgs::RobotState& seed,
                                                         moveit_msgs::RobotState& solution)
{
  using namespace moveit_msgs;
  // error string mappings
  static const std::map<int, std::string> ik_error_mappings = {
      {MoveItErrorCodes::FAILURE, "FAILURE"},
      {MoveItErrorCodes::INVALID_ROBOT_STATE,"INVALID_ROBOT_STATE"},
      {MoveItErrorCodes::FRAME_TRANSFORM_FAILURE,"FRAME_TRANSFORM_FAILURE"},
      {MoveItErrorCodes::INVALID_GROUP_NAME,"INVALID_GROUP_NAME"},
      {MoveItErrorCodes::NO_IK_SOLUTION,"NO_IK_SOLUTION"},
      {MoveItErrorCodes::TIMED_OUT,"TIMED_OUT"}};
  moveit_msgs::MoveItErrorCodes error_code;

  ros::Time time_ik_start = ros::Time::now();
  ros::Duration duration_ik;

  // build ik request
  moveit_msgs::PositionIKRequest ik_req;
  ik_req.group_name = move_group_name_;
  ik_req.avoid_collisions = true;
  ik_req.timeout = ros::Duration(IK_TIMEOUT);
  ik_req.attempts = IK_ATTEMPTS;
  ik_req.pose_stamped = pose;
  ik_req.robot_state = seed;

  // solve ik
  moveit_msgs::GetPositionIK ik_srv;
  ik_srv.request.ik_request = ik_req;
  if (!ik_srv_client_.call(ik_srv))
  {
    ROS_ERROR("IK service call failed for an unknown reason");
    ros::shutdown();
  }

  duration_ik = ros::Time::now() - time_ik_start;
  ROS_DEBUG("IK took %f seconds",duration_ik.toSec());

  // check solution
  error_code.val = ik_srv.response.error_code.val;
  solution = ik_srv.response.solution;
  if(error_code.val != MoveItErrorCodes::SUCCESS)
  {
    std::string err_msg = ik_error_mappings.count(error_code.val) == 0 ? "UNKNOWN" : ik_error_mappings.at(error_code.val);
    std::stringstream ss;
    ss<<pose;
    ROS_DEBUG("IK service failed with code: %s at pose:\n %s", err_msg.c_str(),ss.str().c_str());
  }

  return error_code;
}

moveit_msgs::MoveItErrorCodes PathDataManager::computeFK(const sensor_msgs::JointState &joint_state, geometry_msgs::PoseStamped &pose)
{
  using namespace moveit::core;

  moveit_msgs::MoveItErrorCodes error_code;
  ros::Time time_fk_start = ros::Time::now();
  std::string tip_link_name = moveit_interface_->getEndEffectorLink();

  moveit_msgs::GetPositionFK fk_srv;
  fk_srv.request.header.stamp = ros::Time::now();
  fk_srv.request.header.frame_id = trajectory_frame_;
  fk_srv.request.robot_state.joint_state = joint_state;
  fk_srv.request.fk_link_names = {tip_link_name};
  if (!fk_srv_client_.call(fk_srv))
  {
    ROS_ERROR("FK service call failed for an unknown reason");
    ros::shutdown();
  }

  ros::Duration duration_fk = ros::Time::now() - time_fk_start;
  ROS_DEBUG("FK took %f seconds",duration_fk.toSec());

  // check solution
  error_code.val = fk_srv.response.error_code.val;
  pose = fk_srv.response.pose_stamped[0];
  return error_code;
}

void PathDataManager::toolPoseCallback(const geometry_msgs::PoseStamped &pose_msg)
{
  using namespace moveit_msgs;

  ros::Time time_start = ros::Time::now();

  // transforming pose first
  boost::optional<geometry_msgs::PoseStamped> opt = transformPose(pose_msg,trajectory_frame_);
  if(!opt)
  {
    return;
  }
  const geometry_msgs::PoseStamped& pose = *opt;

  if(equal(prev_pose_,pose.pose,robot_pose_eef_step_.rotation,robot_pose_eef_step_.translation))
  {
    return;
  }
  prev_pose_ = pose.pose;

  // clearing candidate waypoint
  std::lock_guard<std::mutex> lock{candidate_waypoint_mutex_};
  candidate_waypoint_ = boost::none;

  // finding seed state
  moveit_msgs::RobotState seed_state;
  boost::optional<vpr_msgs::TrajectoryWaypoint> latest_waypoint = getLatestWaypoint();
  if(!latest_waypoint.is_initialized())
  {
    auto current_state = getCurrentState();
    if(!current_state)
    {
      return;
    }
    seed_state = *current_state;
  }
  else
  {
    // use previous state
    seed_state = latest_waypoint.get().robot_state;
  }

  // solve ik
  moveit_msgs::RobotState ik_solution;
  moveit_msgs::MoveItErrorCodes error_code = computeIK(pose,seed_state,ik_solution);

  // save waypoint data
  vpr_tracking::StateValidity state_validity = vpr_tracking::StateValidity::OK;
  if (error_code.val == MoveItErrorCodes::SUCCESS)
  {
    latest_valid_state_ = ik_solution;

    // storing candidate waypoint
    candidate_waypoint_ = vpr_msgs::TrajectoryWaypoint();
    candidate_waypoint_->robot_state = ik_solution;
    candidate_waypoint_->tool_pose = pose;
    candidate_waypoint_->raw_tool_pose = pose;
    candidate_waypoint_->snap_mode = (int) current_snap_mode_;

    // check configuration
    if(latest_waypoint.is_initialized() && !withinDistance(candidate_waypoint_->robot_state,
                                                          latest_waypoint.get().robot_state,traj_jump_threshold_))
    {
      state_validity = vpr_tracking::StateValidity::DISTANT_CONFIGURATION;
    }

    ROS_DEBUG_STREAM("Computed new candidate waypoint for robot tool pose:\n"<<pose.pose);
  }
  else
  {
    state_validity = vpr_tracking::StateValidity::INFEASIBLE;
  }
  coloringState(state_validity);
  ros::Duration duration_total = ros::Time::now() - time_start;
  ROS_DEBUG("Tool pose callback total took %f seconds to complete",duration_total.toSec());
}

void PathDataManager::jointStateCallback(const sensor_msgs::JointState& msg)
{
  using namespace moveit_msgs;

  ros::Time time_start = ros::Time::now();
  current_joint_state_ = msg;

  // solve fk
  geometry_msgs::PoseStamped pose;
  moveit_msgs::MoveItErrorCodes error_code = computeFK(current_joint_state_, pose);

  // save waypoint data
  vpr_tracking::StateValidity state_validity = vpr_tracking::StateValidity::OK;
  if (error_code.val == MoveItErrorCodes::SUCCESS)
  {
    latest_valid_state_.joint_state = current_joint_state_;

    // storing candidate waypoint
    candidate_waypoint_ = vpr_msgs::TrajectoryWaypoint();
    candidate_waypoint_->robot_state = latest_valid_state_;
    candidate_waypoint_->tool_pose = pose;

    // check configuration
    boost::optional<vpr_msgs::TrajectoryWaypoint> latest_waypoint = getLatestWaypoint();
    if(latest_waypoint.is_initialized() && !withinDistance(candidate_waypoint_->robot_state,
                                                          latest_waypoint.get().robot_state,traj_jump_threshold_))
    {
      state_validity = vpr_tracking::StateValidity::DISTANT_CONFIGURATION;
    }

    ROS_DEBUG_STREAM("Computed new candidate waypoint for robot tool pose:\n"<<pose.pose);
  }
  else
  {
    ROS_ERROR("Forward kinematics failed, moveit error code: %i",error_code.val);
  }
  coloringState(state_validity);
  ros::Duration duration_total = ros::Time::now() - time_start;
  ROS_DEBUG("Joint State callback took %f seconds to complete",duration_total.toSec());
}

void PathDataManager::coloringState(const vpr_tracking::StateValidity &state_validity)
{
  // coloring state
  moveit_msgs::DisplayRobotState disp_msg;
  disp_msg.state = latest_valid_state_;
  if(state_validity != vpr_tracking::StateValidity::OK)
  {
    std_msgs::ColorRGBA color_msg;
    switch(state_validity)
    {
      case vpr_tracking::StateValidity::DISTANT_CONFIGURATION:
        std::tie(color_msg.r, color_msg.g, color_msg.b, color_msg.a) = robot_state_colors::CONFIGURATION_CHANGED;
        break;
      case vpr_tracking::StateValidity::INFEASIBLE:
        std::tie(color_msg.r, color_msg.g, color_msg.b, color_msg.a) = robot_state_colors::INFEASIBLE;
        break;
      default:
        break;
    }

    const std::vector<std::string>& link_names = moveit_interface_->getLinkNames();
    for(const std::string& ln : link_names)
    {
      moveit_msgs::ObjectColor obj_color;
      obj_color.id = ln;
      obj_color.color = color_msg;
      disp_msg.highlight_links.push_back(obj_color);
    }
  }

  setDispRobotStateMsg(disp_msg);
}

boost::optional<vpr_msgs::TrajectoryWaypoint> PathDataManager::getLatestWaypoint() const
{
  if(traj_segments_buffer_.empty())
  {
    return boost::none;
  }

  const TrajectorySegmentInfo& ti = traj_segments_buffer_.back();
  return boost::make_optional(!ti.segment.waypoints.empty(),ti.segment.waypoints.back());
}

void PathDataManager::updatePathWaypointMarkers()
{
  waypoints_markers_.markers.clear();
  int marker_id = 0;
  for(const TrajectorySegmentInfo& s: traj_segments_buffer_)
  {
    const std::vector<vpr_msgs::TrajectoryWaypoint>& waypoins = s.segment.waypoints;
    if(waypoins.empty())
    {
      continue;
    }
    std::string frame_id = waypoins.front().tool_pose.header.frame_id;

    visualization_msgs::MarkerArray markers = createPathLineMarker(frame_id,
                                                               s.segment.waypoints,
                                                               getRecordingModeColor(s.recording_mode),++marker_id);
    waypoints_markers_.markers.insert(waypoints_markers_.markers.end(),markers.markers.begin(),markers.markers.end());

    if(s.recording_mode == RecordingModes::WAYPOINT)
    {
      markers = createPathDotsMarker(frame_id,s.segment.waypoints,RECORDING_MODE_WAYPOINT_DOT_RGBA,++marker_id);
      waypoints_markers_.markers.insert(waypoints_markers_.markers.end(),markers.markers.begin(),markers.markers.end());
    }
  }
}

jsk_rviz_plugins::OverlayText PathDataManager::createInfoOverlay() const
{
  std::lock_guard<std::mutex> lock{candidate_waypoint_mutex_};

  jsk_rviz_plugins::OverlayText msg;
  msg.action = jsk_rviz_plugins::OverlayText::ADD;
  msg.top = 10;
  msg.left = 10;
  msg.width = 500;
  msg.height = 300;
  msg.text_size = 20;

  msg.fg_color.r = 1.0;
  msg.fg_color.g = 1.0;
  msg.fg_color.b = 1.0;
  msg.fg_color.a = 0.8;

  msg.bg_color.r = 1.0;
  msg.bg_color.g = 1.0;
  msg.bg_color.b = 1.0;
  msg.bg_color.a = 0.0;

  int num_segments = traj_segments_buffer_.size();
  int num_waypoints = std::accumulate(traj_segments_buffer_.begin(), traj_segments_buffer_.end(),
      0,
      [](int sum, auto& elem) {return sum + elem.segment.waypoints.size();}
  );

  std::ostringstream ss;
  ss << "Trajectory segments: " << num_segments << std::endl;
  ss << "Number of waypoints: " << num_waypoints << std::endl;
  ss << "Snap Mode: " << to_string(current_snap_mode_) << std::endl;
  msg.text = ss.str();
  return msg;
}

bool PathDataManager::withinDistance(const moveit_msgs::RobotState& st1_msg, const moveit_msgs::RobotState& st2_msg,
                                     const JumpThreshold& max_distance)
{
  using namespace moveit::core;

  moveit::core::RobotModelConstPtr model = moveit_interface_->getRobotModel();
  moveit::core::RobotState st1(model), st2(model);
  robot_state::robotStateMsgToRobotState(st1_msg,st1);
  robot_state::robotStateMsgToRobotState(st2_msg,st2);

  const std::vector<const JointModel*>& joint_models = st1.getRobotModel()->getActiveJointModels();
  for(const JointModel* jm : joint_models)
  {

    double d = st1.distance(st2,jm);
    switch(jm->getType())
    {
      case JointModel::PRISMATIC:

        if(d > max_distance.prismatic)
        {
          return false;
        }
        break;

      case JointModel::REVOLUTE:
        if(d > max_distance.revolute)
        {
          return false;
        }
        break;
    }
  }
  return true;
}

moveit_msgs::RobotStatePtr PathDataManager::getCurrentState(ros::Duration max_delay)
{
  using namespace moveit_msgs;
  GetPlanningSceneRequest req;
  GetPlanningSceneResponse res;
  req.components.components = GetPlanningSceneRequest::_components_type::ROBOT_STATE;
  if(!get_planning_scene_client_.call(req, res))
  {
    ROS_ERROR("Call to service %s failed",get_planning_scene_client_.getService().c_str());
    return nullptr;
  }
  moveit_msgs::RobotStatePtr rs = boost::make_shared<moveit_msgs::RobotState>(res.scene.robot_state);

  // updating internal robot state
  robot_state::robotStateMsgToRobotState(res.scene.robot_state,*current_robot_st_,true);

  return std::move(rs);
}

void PathDataManager::setDispRobotStateMsg(const moveit_msgs::DisplayRobotState& st)
{
  std::lock_guard<std::mutex> lock(disp_robot_st_mutex_);
  if(!disp_robot_st_)
  {
    disp_robot_st_.reset(new moveit_msgs::DisplayRobotState(st));
  }
  else
  {
    *disp_robot_st_ = st;
  }
}

moveit_msgs::DisplayRobotState PathDataManager::getDispRobotStateMsg()
{
  std::lock_guard<std::mutex> lock(disp_robot_st_mutex_);
  return *disp_robot_st_;
}

} /* namespace vpr_tracking */
