/**
 * \file tool_tracking_node.cpp
 * \copyright (c) 2018 Southwest Research Institute
 */
#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <dynamic_reconfigure/server.h>

#include "vpr_tracking/tool.h"
#include "vpr_tracking/tool_tracker.h"
#include "vpr_tracking/ToolTrackerConfig.h"

static const std::string TOOL_POSE_TOPIC = "tool_pose";
static const std::string MARKERS_TOPIC = "markers";
static const std::string TOOL_DESCRIPTION_PARAM = "tool_description";
static const std::string MARKER_PREFIX = "aruco_";

using EigenTf = Eigen::Isometry3d;

using ReconfigureServer = dynamic_reconfigure::Server<vpr_tracking::ToolTrackerConfig>;

namespace vpr_tracking
{

/**
 * @details Uses exponential moving average for position and cumulative slerp for the rotation portion
 *        in order to compute a pose.  The reference links are as follows:
 *        - https://en.wikipedia.org/wiki/Moving_average
 *        - https://en.wikipedia.org/wiki/Slerp
 *        - https://math.stackexchange.com/questions/1204228/how-would-i-apply-an-exponential-moving-average-to-quaternions
 * @param poses The poses to be smoothed out ordered from oldest to latest
 * @param alpha The degree of weight decrease
 * @return  The computed pose
 */
EigenTf computeAveragePose(const std::vector<EigenTf>& poses, double alpha);

class ToolTrackerNode
{
public:
  ToolTrackerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    markers_sub_ = nh.subscribe(MARKERS_TOPIC, 10, &ToolTrackerNode::markersCallback, this);
    tool_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(TOOL_POSE_TOPIC, 10);
    tool_pose_srv_ = nh.advertiseService("get_tool_pose", &ToolTrackerNode::toolPoseSrvCallback, this);

    std::string tool_description; //note: not in the private node namespace
    if (! nh.getParam(TOOL_DESCRIPTION_PARAM, tool_description))
      throw std::runtime_error("Required parameter '" + TOOL_DESCRIPTION_PARAM + "' not found");

    // setting up dynamic reconfigure server
    ReconfigureServer::CallbackType callback = boost::bind(&ToolTrackerNode::reconfigureCallback,this, _1, _2);
    reconfigure_server_.setCallback(callback);

    poses_buffer_.clear();

    tool_ = std::unique_ptr<vpr_tracking::Tool>(new vpr_tracking::Tool(tool_description, MARKER_PREFIX));
    //tool_pose_timer_ = nh.createTimer(ros::Duration(1/calc_rate), &ToolTrackerNode::toolPoseTimerCallback, this);
  }

  void spin()
  {
    ros::spin();
  }

  void markersCallback(aruco_msgs::MarkerArray::ConstPtr msg)
  {
    latest_markers_.clear();
    for (auto &m : msg->markers)
    {
      latest_markers_.push_back(tool_tracking::Marker{
          .header = msg->header,
          .id = (int) m.id,
          .pose = utils::poseToEigenTransform<double, Eigen::Isometry>(m.pose)
      });
    }
    broadcastMarkerTransforms(latest_markers_);
    toolPose(latest_markers_);
  }

  bool toolPoseSrvCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &)
  {
    toolPose(latest_markers_);
    return true;
  }

  void toolPose(const std::vector<tool_tracking::Marker> &markers)
  {
    tool_tracking::PoseCorrespondences correspondences;

    std::vector<tool_tracking::Marker> valid_markers = filterMarkers(markers, *tool_);
    if (valid_markers.size() < tracking_config_.min_num_markers)
    {
      ROS_DEBUG_NAMED("detailed", "Saw %zu markers, need at least %i",
          valid_markers.size(), tracking_config_.min_num_markers);
      return;
    }

    for (auto& marker : valid_markers)
    {
      geometry_msgs::PoseStamped ref_pose;
      if (tool_->markerPose(marker.id, ref_pose))
      {
        correspondences.push_back(tool_tracking::PoseCorrespondence{
            utils::poseToEigenTransform<double, Eigen::Isometry>(ref_pose.pose),
            marker.pose
        });
      }
    }

    boost::optional<EigenTf> tool_pose = latest_pose_ ?
        tool_tracking::calculateToolPose(correspondences, *latest_pose_, tracking_config_) :
        tool_tracking::calculateToolPose(correspondences, tracking_config_);

    if (! tool_pose)
    {
      latest_pose_ = boost::none;
      ROS_DEBUG("Couldn't estimate tracking tool pose from markers");
      return;
    }

    EigenTf curr_tool_pose = *tool_pose;

    auto pos = curr_tool_pose.translation();
    auto euler = curr_tool_pose.rotation().eulerAngles(0,1,2);
    ROS_DEBUG_NAMED("detailed","Estimated Tool pose: (x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f)",
        pos.x(), pos.y(), pos.z(), euler(0), euler(1), euler(2));

    if (! poses_buffer_.empty())
    {
      std::vector<EigenTf> poses(poses_buffer_.begin(), poses_buffer_.end());
      poses.push_back(*tool_pose);
      curr_tool_pose = computeAveragePose(poses, tracking_config_.weight_coeff);
    }

    // saving pose
    latest_pose_ = curr_tool_pose;
    latest_time_ = valid_markers.at(0).header.stamp;
    savePose(curr_tool_pose);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = valid_markers.at(0).header;
    pose_msg.header.stamp = ros::Time::now();
    tf::poseEigenToMsg(curr_tool_pose, pose_msg.pose);

    tool_pose_pub_.publish(pose_msg);

    geometry_msgs::TransformStamped camera_to_tool_t = utils::poseToTransform(pose_msg);
    camera_to_tool_t.child_frame_id = tool_->getFrameID();
    tf_broadcaster_.sendTransform(camera_to_tool_t);
  }

  void broadcastMarkerTransforms(const std::vector<tool_tracking::Marker> markers)
  {
    for (auto& marker : markers)
    {
      geometry_msgs::PoseStamped marker_pose;
      marker_pose.header = marker.header;
      tf::poseEigenToMsg(marker.pose, marker_pose.pose);

      geometry_msgs::TransformStamped marker_transform_msg = utils::poseToTransform(marker_pose);
      marker_transform_msg.child_frame_id = "real_aruco" + std::to_string(marker.id);
      tf_broadcaster_.sendTransform(marker_transform_msg);
    }
  }

  void savePose(const EigenTf& pose)
  {
    poses_buffer_.push_back(pose);
    prunePosesBuffer(tracking_config_.averaging_buffer_size);
  }

  void prunePosesBuffer(int num_elements)
  {
    while(poses_buffer_.size() > num_elements)
    {
      poses_buffer_.pop_front();
    }
  }

  void reconfigureCallback(vpr_tracking::ToolTrackerConfig& config, uint32_t level)
  {
    tracking_config_ = config;
    prunePosesBuffer(tracking_config_.averaging_buffer_size);

  }

private:
  ros::Publisher tool_pose_pub_;
  ros::Subscriber markers_sub_;
  ros::Timer tool_pose_timer_;
  ros::ServiceServer tool_pose_srv_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::vector<tool_tracking::Marker> latest_markers_;
  std::unique_ptr<Tool> tool_;
  ros::Time latest_time_;
  boost::optional<EigenTf> latest_pose_;

  // pose noise smoothing/filtering parameters
  vpr_tracking::ToolTrackerConfig tracking_config_;
  std::list<EigenTf> poses_buffer_;

  // dynamic reconfigure
  ReconfigureServer reconfigure_server_;

};

EigenTf computeAveragePose(const std::vector<EigenTf>& poses, double alpha)
{
  using namespace Eigen;

  Eigen::Vector3d nominator = Vector3d::Zero();
  double denominator = 0.0;
  Quaterniond q_prev = Quaterniond::Identity();
  Quaterniond q_f;
  for(std::size_t i = 0 ; i < poses.size(); i++)
  {
    // using exponential moving average for positions
    const auto& pose = poses[i];
    int power = poses.size() - 1 - i;
    double bottom = std::pow(1 - alpha,power);
    nominator +=bottom * pose.translation();
    denominator +=bottom;

    // using slerp for quaternions
    if(i == 0)
    {
      q_f = poses[i].rotation().matrix();
      continue;
    }

    q_prev = q_f;
    q_f = pose.rotation().matrix();
    q_f = q_prev.slerp(alpha,q_f);

  }
  Eigen::Vector3d pos = nominator/denominator;

  EigenTf smoothed_pose = Translation3d(pos) * q_f;
  return std::move(smoothed_pose);
}

} //namespace vpr_tracking

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "tool_tracking_node");
  ros::NodeHandle nh, pnh("~");
  try
  {
    vpr_tracking::ToolTrackerNode tracker(nh, pnh);
    tracker.spin();
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR_STREAM("Tool tracker: " << ex.what());
    return -1;
  }
  return 0;
}
