/*
 * plan_toolpath.h
 *
 *  Created on: Jun 1, 2021
 *      Author: jnicho
 */

#ifndef INCLUDE_VPR_TRAJECTORY_STAGES_PLAN_TOOLPATH_H_
#define INCLUDE_VPR_TRAJECTORY_STAGES_PLAN_TOOLPATH_H_

#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <eigen_stl_containers/eigen_stl_containers.h>

namespace vpr_trajectory
{
namespace stages
{

class PlanToolpath : public moveit::task_constructor::PropagatingEitherWay
{
public:
  PlanToolpath(const std::string& stage_id,
               const std::string& group_name);
  virtual ~PlanToolpath();

  void init(const moveit::core::RobotModelConstPtr& robot_model) override;

  void computeForward(const moveit::task_constructor::InterfaceState& from) override;
  void computeBackward(const moveit::task_constructor::InterfaceState& to) override;

  void setToolpath(const EigenSTL::vector_Isometry3d& toolpath);

};

} /* namespace stages */
} /* namespace vpr_trajectory */

#endif /* INCLUDE_VPR_TRAJECTORY_STAGES_PLAN_TOOLPATH_H_ */
