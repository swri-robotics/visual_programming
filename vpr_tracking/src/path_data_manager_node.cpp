/**
 * \file path_data_manager_node.cpp
 * \copyright (c) 2018 Southwest Research Institute
 */

#include <vpr_msgs/EditPathWaypoints.h>
#include <vpr_tracking/path_data_manager.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <boost/functional.hpp>
#include <boost/format.hpp>

static const std::string START_PATH_PREVIEW_SERVICE = "start_path_preview";
static const std::string STOP_PATH_PREVIEW_SERVICE = "stop_path_preview";
static const std::string EXECUTE_PATH_SERVICE = "execute_path";
static const std::string MOVE_HOME_SERVICE = "move_home";
static const std::string EDIT_PATH_WAYPOINTS_SERVICE = "edit_path_waypoints";
static const double SM_WAIT_DURATION = 2.0;

namespace vpr_tracking
{

enum class SnapMethod : int
{
  None = 0,
  Point = 1,
  Line = 2,
  Face = 3
};

class PathManagerExecutive
{
public:
  PathManagerExecutive(ros::NodeHandle &nh, ros::NodeHandle &nhp)
  {
    path_manager_.reset(new PathDataManager(nh,nhp));
    // init ROS
    edit_path_srv_ = nh.advertiseService(EDIT_PATH_WAYPOINTS_SERVICE,&PathManagerExecutive::editPathWaypointsCallback,this);
    start_path_preview_srv_ = nh.advertiseService(START_PATH_PREVIEW_SERVICE,
                                                  &PathManagerExecutive::startPreviewServiceCallback,this);
    stop_path_preview_srv_ = nh_.advertiseService(STOP_PATH_PREVIEW_SERVICE,
                                                  &PathManagerExecutive::stopPreviewServiceCallback,this);
    execute_path_srv_ = nh.advertiseService(EXECUTE_PATH_SERVICE,&PathManagerExecutive::executeServiceCallback,this);
    move_home_srv_ = nh.advertiseService(MOVE_HOME_SERVICE,&PathManagerExecutive::moveHomeServiceCallback,this);

    // start tracking
    path_manager_->startPoseTracking();
  }

  ~PathManagerExecutive() = default;

protected:
  bool editPathWaypointsCallback(vpr_msgs::EditPathWaypointsRequest& req, vpr_msgs::EditPathWaypointsResponse& res)
  {
    using namespace vpr_msgs;
    using EditT = EditPathWaypointsRequest;

    res.success = false;
    switch(req.operation)
    {
      case EditT::ADD:
        res.success = path_manager_->addWaypoint();
        break;
      case EditT::DELETE:
        res.success = path_manager_->removeLastWaypoint();
        break;
      case EditT::CHANGE_SNAP_MODE:
        res.success = path_manager_->changeSnapMode();
        break;
      case EditT::CLEAR_ALL:
        res.success = path_manager_->clearAllWaypoints();
        break;
      case EditT::CONTINUOUS_START:
        res.success = path_manager_->startContinuousSegment();
        break;
      case EditT::CONTINUOUS_STOP:
        res.success = path_manager_->stopContinuousSegment();
        break;
      case EditT::CONTINUOUS_CANCEL:
        res.success = path_manager_->cancelContinuousSegment();
        break;
      default:
        res.success = false;
        res.msg = boost::str(boost::format("Action '%1%' is not supported") % req.operation);
    }

    return true;
  }

  bool startPreviewServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    boost::optional<moveit_msgs::DisplayTrajectory> disp_traj = path_manager_->planRobotTrajectories();
    res.success = true;
    if(!disp_traj.is_initialized())
    {
      res.success = false;
      return true;
    }

    path_manager_->stopPoseTracking();
    path_manager_->stopPathPreview();

    path_manager_->startPathPreview(disp_traj.get());

    path_manager_->startPoseTracking();

    return true;
  }

  bool stopPreviewServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    path_manager_->stopPathPreview();
    return true;
  }

  bool executeServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    boost::optional<moveit_msgs::DisplayTrajectory> disp_traj = path_manager_->planRobotTrajectories();
    res.success = false;
    if(!disp_traj.is_initialized())
    {
      res.success = false;
      return true;
    }

    path_manager_->stopPoseTracking();
    path_manager_->stopPathPreview();

    // executing path
    res.success = path_manager_->executePath(disp_traj.get());

    path_manager_->startPoseTracking();

    return true;
  }

  bool moveHomeServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    path_manager_->stopPoseTracking();
    res.success = path_manager_->moveHome();
    path_manager_->startPoseTracking();
    return true;
  }

  // path manager
  std::shared_ptr<PathDataManager> path_manager_;

  // ROS
  ros::NodeHandle nh_;
  ros::ServiceServer edit_path_srv_;
  ros::ServiceServer start_path_preview_srv_;
  ros::ServiceServer stop_path_preview_srv_;
  ros::ServiceServer execute_path_srv_;
  ros::ServiceServer move_home_srv_;
};

}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "path_data_manager_node");
  ros::NodeHandle nh, nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  try
  {
    vpr_tracking::PathManagerExecutive node(nh, nhp);
    ros::waitForShutdown();
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }


  return 0;
}
