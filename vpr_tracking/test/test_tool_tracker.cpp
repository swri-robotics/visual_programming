#include <ros/ros.h>
#include <vpr_tracking/tool_tracker.h>
#include <vpr_tracking/ToolTrackerConfig.h>

using namespace vpr_tracking;
using namespace Eigen;

using EigenTf = Eigen::Isometry3d;

static const Translation3d TOOL_TRANS = {1.0, 0.5, 0.0};
static const Quaterniond TOOL_QUAT = {0.966, 0.0, 0.259, 0.0};

static const std::vector<Translation3d> MARKERS_REFS = {
  Translation3d(0.0, 0.0, 0.0),
  Translation3d(0.1, 0.0, 0.0),
  Translation3d(0.1, 0.1, 0.0),
  Translation3d(0.0, 0.1, 0.0)
};

tool_tracking::PoseCorrespondences makeCorrespondences();

int main(int argc, char * argv[])
{
  tool_tracking::PoseCorrespondences corrs = makeCorrespondences();
  ToolTrackerConfig config;

  boost::optional<EigenTf> tool_pose_est = tool_tracking::calculateToolPose(corrs, config);

  if(!tool_pose_est)
  {
    std::cout<<"Could not estimate tool pose"<<std::endl;
  }

  EigenTf tool_pose = *tool_pose_est;
  auto position = tool_pose.translation();
  auto orient = Quaterniond(tool_pose.rotation());

  std::cout << "Estimated tool pose: " << std::endl;
  std::cout << "  t: (";
  std::cout << std::setw(12) << position.x() << ", ";
  std::cout << std::setw(12) << position.y() << ", ";
  std::cout << std::setw(12) << position.z() << ")";
  std::cout << "  q: (";
  std::cout << std::setw(12) << orient.w() << ", ";
  std::cout << std::setw(12) << orient.x() << ", ";
  std::cout << std::setw(12) << orient.y() << ", ";
  std::cout << std::setw(12) << orient.z() << ")";
  std::cout << std::endl;
  std::cout << "Actual tool pose: " << std::endl;
  std::cout << "  t: (";
  std::cout << std::setw(12) << TOOL_TRANS.x() << ", ";
  std::cout << std::setw(12) << TOOL_TRANS.y() << ", ";
  std::cout << std::setw(12) << TOOL_TRANS.z() << ")";
  std::cout << "  q: (";
  std::cout << std::setw(12) << TOOL_QUAT.w() << ", ";
  std::cout << std::setw(12) << TOOL_QUAT.x() << ", ";
  std::cout << std::setw(12) << TOOL_QUAT.y() << ", ";
  std::cout << std::setw(12) << TOOL_QUAT.z() << ")";
  std::cout << std::endl;

  return 0;
}

tool_tracking::PoseCorrespondences makeCorrespondences()
{
  EigenTf tool_tf = TOOL_TRANS * TOOL_QUAT;

  tool_tracking::PoseCorrespondences corrs;
  for (auto & ref : MARKERS_REFS)
  {
    tool_tracking::PoseCorrespondence corr;
    corr.reference = ref;

    EigenTf meas = tool_tf * ref;
    corr.measured = meas;

    std::printf("Ref: (%f, %f, %f) --> Meas: (%f, %f, %f)\n",
        ref.x(), ref.y(), ref.z(),
        meas.translation().x(), meas.translation().y(), meas.translation().z()
    );

    corrs.push_back(corr);
  }
  return corrs;
}
