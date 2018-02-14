/**
 * \file tool_tracker.cpp
 * \copyright (c) 2018 Southwest Research Institute
 */
#include "vpr_tracking/tool_tracker.h"

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen_conversions/eigen_msg.h>

static const int CERES_MAX_INVALID_STEPS = 100;

inline double radToDeg(double x) {return x * 180.0/3.141592635;}
inline double degToRad(double x) {return x * 3.141592635*180.0;}

namespace vpr_tracking
{

namespace tool_tracking
{

/**
 * \brief Cost functor used by Ceres AutoDiffCostFunction. Computes the cost of a single pose correspondence.
 */
class PoseCorrespondenceCost
{
public:
  PoseCorrespondenceCost(const PoseCorrespondence &corr) : correspondence_(corr) {}

  template <typename Scalar>
  bool operator()(const Scalar * const translation, const Scalar * const rotation, Scalar * residuals) const
  {
    using AutoDiffVec3T = Eigen::Matrix<Scalar, 3, 1>;
    using AutoDiffTranslationT = Eigen::Translation<Scalar,3>;
    using AutoDiffTransformT = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
    using AutoDiffQuatT = Eigen::Quaternion<Scalar>;
    using AutoDiffRotMatT = Eigen::Matrix<Scalar, 3, 3>;

    //The measured transform for this correspondence
    AutoDiffTransformT meas_t = correspondence_.measured.cast<Scalar>();

    //The current expected transform the solver is considering
    AutoDiffTranslationT eigen_trans(translation[0], translation[1], translation[2]);
    AutoDiffQuatT eigen_quat(rotation[0], rotation[1], rotation[2], rotation[3]);
    AutoDiffTransformT curr_cam_to_tool = eigen_trans * eigen_quat;

    AutoDiffTransformT ref_t = correspondence_.reference.cast<Scalar>();
    AutoDiffTransformT expect_t = curr_cam_to_tool * ref_t;

    AutoDiffVec3T trans_diff = meas_t.translation() - expect_t.translation();
    AutoDiffRotMatT rel_rot = meas_t.rotation().inverse() * expect_t.rotation();

    //ceres creates an autodiffable angle-axis where the angle is the norm of the vector
    Scalar rel_aa[3];
    ceres::RotationMatrixToAngleAxis(rel_rot.data(), rel_aa);
    //Scalar angle_diff = ceres::sqrt(rel_aa[0]*rel_aa[0] + rel_aa[1]*rel_aa[1] + rel_aa[2]*rel_aa[2]);

    residuals[0] = trans_diff.x();
    residuals[1] = trans_diff.y();
    residuals[2] = trans_diff.z();
    residuals[3] = rel_aa[0];
    residuals[4] = rel_aa[1];
    residuals[5] = rel_aa[2];
    return true;
  }

  static ceres::CostFunction * Create(const PoseCorrespondence &corr)
  {
    //6 residuals, 3-element translation and 4-element quaternion parameter blocks
    return new ceres::AutoDiffCostFunction<PoseCorrespondenceCost,6,3,4>(new PoseCorrespondenceCost(corr));
  }

private:
  //This object is temporary and only keeps a reference to the data it uses
  const PoseCorrespondence & correspondence_;
};

/**
 * @brief Check visibility heuristic for observed marker
 */
bool checkObservability(const Marker &marker)
{
  //Should have positive Z relative to camera
  if (marker.pose.translation().z() < 0)
    return false;

  //Rotate the marker z axis by marker orientation
  auto zvec = Eigen::Vector3d::UnitZ();
  auto rot = Eigen::AngleAxisd(marker.pose.rotation());

  auto axis = rot.axis();
  auto theta = rot.angle();
  auto rotvec = zvec*cos(theta) + (axis.cross(zvec))*sin(theta) + axis*(axis.dot(zvec))*(1-cos(theta));

  ROS_DEBUG_STREAM_NAMED("detailed", "    marker normal z-component: " << rotvec.z());

  //The rotated vector should have a -z component or it would be facing away from the camera
  return rotvec.z() < 0;
}

std::vector<Marker> filterMarkers(const std::vector<Marker> &markers, const Tool &tool)
{
  std::vector<Marker> valid_markers;
  std::copy_if(
    markers.begin(), markers.end(),
    std::back_inserter(valid_markers),
    [&](const auto& marker)
    {
      auto euler = marker.pose.rotation().eulerAngles(0,1,2);
      ROS_DEBUG_NAMED("detailed", "Marker %i: [x = %f, y = %f, z = %f, rx = %f, ry = %f, rz = %f]",
          marker.id,
          marker.pose.translation().x(),
          marker.pose.translation().y(),
          marker.pose.translation().z(),
          radToDeg(euler(0)),
          radToDeg(euler(1)),
          radToDeg(euler(2)));
      if (! tool.hasMarker(marker.id))
      {
        ROS_DEBUG_NAMED("detailed","Filtering marker %i which was not found in tracking tool", marker.id);
        return false;
      }
      if (! checkObservability(marker))
      {
        ROS_DEBUG_NAMED("detailed","Filtering marker %i which should not be visible with observed orientation",
            marker.id);
        return false;
      }
      return true;
    }
  );
  return valid_markers;
}

Eigen::Isometry3d initialGuess(const PoseCorrespondence &corr)
{
  using EigenTf = Eigen::Isometry3d;
  EigenTf guess = corr.measured * corr.reference.inverse();
  return guess;
}

boost::optional<Eigen::Isometry3d> calculateToolPose(
    const PoseCorrespondences &pose_corrs,
    const Eigen::Isometry3d &init_pose,
    const ToolTrackerConfig &config)
{
  double translation[] = {init_pose.translation().x(), init_pose.translation().y(), init_pose.translation().z()};
  auto quat = Eigen::Quaterniond(init_pose.rotation());
  double rotation[] = {quat.w(), quat.x(), quat.y(), quat.z()};

  ceres::Problem problem;
  problem.AddParameterBlock(translation, 3);
  problem.AddParameterBlock(rotation, 4, new ceres::QuaternionParameterization);

  for (const auto & correspondence : pose_corrs)
  {
    //pose correspondences have 6 residuals:
    //  - x, y, and z deviations
    //  - the 3 elements of the angle-axis rotation between the two poses (axis vector scaled by angle)
    problem.AddResidualBlock(
        PoseCorrespondenceCost::Create(correspondence),
        nullptr,
        translation,
        rotation);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_type = ceres::TRUST_REGION;
  options.max_num_consecutive_invalid_steps = CERES_MAX_INVALID_STEPS;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  ROS_DEBUG_STREAM_NAMED("optimizer", summary.FullReport());

  if (! summary.IsSolutionUsable())
  {
    ROS_DEBUG_STREAM_NAMED("detailed", "Optimizer solution unusable");
    return boost::none;
  }

  const double max_final_cost = std::pow(10.0, -config.max_allowed_final_cost);
  if (summary.final_cost > max_final_cost)
  {
    ROS_DEBUG_NAMED("detailed", "Final optimization cost %f higher than limit of %f",
        summary.final_cost, max_final_cost);
    return boost::none;
  }

  Eigen::Isometry3d tool_pose =
    Eigen::Translation3d(translation[0], translation[1], translation[2]) *
    Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]);
  return tool_pose;
}

//Compute with a full optimization
boost::optional<Eigen::Isometry3d> calculateToolPose(const PoseCorrespondences &pose_corrs, const ToolTrackerConfig &config)
{
  Eigen::Isometry3d init_pose = initialGuess(pose_corrs.at(0));
  return calculateToolPose(pose_corrs, init_pose, config);
}

}

}
