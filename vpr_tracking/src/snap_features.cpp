#include <vpr_tracking/snap_features.h>
#include <numeric>
#include <Eigen/Geometry>
#include <ros/console.h>

namespace vpr_tracking
{

std::unique_ptr<SnapableFeature> snapFeatureFromParam(XmlRpc::XmlRpcValue &param)
{
  if (! param.hasMember("type"))
    throw XmlRpc::XmlRpcException("don't have key 'type'");
  std::string type = static_cast<std::string>(param["type"]);
  if (! param.hasMember("dist"))
    throw XmlRpc::XmlRpcException("don't have key 'dist'");
  double dist = static_cast<double>(param["dist"]);
  if (! param.hasMember("name"))
    throw XmlRpc::XmlRpcException("don't have key 'name'");
  std::string name = static_cast<std::string>(param["name"]);
  std::vector<Eigen::Vector3d> pts;
  XmlRpc::XmlRpcValue points_elem = param["points"];
  for (int i=0; i < points_elem.size(); ++i)
  {
    auto pt = points_elem[i];
    pts.emplace_back(pt[0], pt[1], pt[2]);
  }

  std::unique_ptr<SnapableFeature> feature;
  if (type == "Point")
  {
    if (pts.size() != 1)
      throw std::runtime_error("Point feature requires exactly 1 point");
    feature = std::make_unique<PointFeature>(name, pts.at(0), dist);
  }
  else if (type == "Line")
  {
    if (pts.size() != 2)
      throw std::runtime_error("Line feature requires exactly 2 points");
    feature = std::make_unique<LineFeature>(name, pts.at(0), pts.at(1), dist);
  }
  else if (type == "ConvexFace")
  {
    if (pts.size() < 3)
      throw std::runtime_error("ConvexFace feature requires at least 3 points");
    feature = std::make_unique<ConvexFaceFeature>(name, pts, dist);
  }
  else
    throw std::runtime_error("Unknown snap feature requested: " + type);

  ROS_INFO("  feature: name = %s, type = %s, dist = %f", name.c_str(), type.c_str(), dist);
  return feature;
}

boost::optional<Eigen::Vector3d> PointFeature::snap(const Eigen::Vector3d &pt) const
{
  Eigen::Vector3d dv = pt - center_;
  if (dv.norm() <= dist_)
    return center_;
  return boost::none;
}

visualization_msgs::Marker PointFeature::visualization() const
{
  visualization_msgs::Marker marker;
  marker.ns = "snap_feature_" + name_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center_.x();
  marker.pose.position.y = center_.y();
  marker.pose.position.z = center_.z();
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = dist_;
  marker.scale.y = dist_;
  marker.scale.z = dist_;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

boost::optional<Eigen::Vector3d> LineFeature::snap(const Eigen::Vector3d &pt) const
{
  using namespace Eigen;
  //compute projection of point onto infinite line
  Vector3d p1x = pt - p1_;
  Vector3d p12 = p2_ - p1_;
  Vector3d pp = p1_ + (p1x.dot(p12)/p12.dot(p12)) * p12;

  //compute parameter of projected point to see where it lies on the segment
  Vector3d p1p = pp - p1_;
  double c = p1p.norm() / p12.norm();
  if (p1p.dot(p12) < 0)
    c = -c;

  if (c < 0) //behind first segment point, may snap directly to segment end
  {
    if (p1x.norm() < dist_)
      return p1_;
  }
  else if (c > 1) //after last segment point, may snap directly to segment end
  {
    Vector3d p2x = pt - p2_;
    if (p2x.norm() < dist_)
      return p2_;
  }
  else //somewhere along the segment, may snap to the projected point
  {
    Vector3d ppx = pt - pp;
    if (ppx.norm() < dist_)
      return pp;
  }
  return boost::none;
}

visualization_msgs::Marker LineFeature::visualization() const
{
  auto eigen_to_geometry_msgs = [](const Eigen::Vector3d& pt) -> geometry_msgs::Point
  {
    geometry_msgs::Point gpt;
    gpt.x = pt.x();
    gpt.y = pt.y();
    gpt.z = pt.z();
    return gpt;
  };

  visualization_msgs::Marker marker;
  marker.ns = "snap_feature_" + name_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.push_back(eigen_to_geometry_msgs(p1_));
  marker.points.push_back(eigen_to_geometry_msgs(p2_));
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = dist_;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

ConvexFaceFeature::ConvexFaceFeature(const std::string &name, const std::vector<Eigen::Vector3d> &points, double dist) :
  name_(name),
  pts_(points),
  dist_(dist)
{
  //TODO perform check if these points are actually coplanar and form a convex polygon
  using namespace Eigen;
  if (pts_.size() < 3)
    throw std::runtime_error("Cannot create face feature with fewer than 3 points");
  Vector3d v01 = pts_[1] - pts_[0];
  Vector3d v02 = pts_[2] - pts_[1];
  plane_normal_ = v01.cross(v02).normalized();

  Vector3d sum_pts = std::accumulate(pts_.begin(), pts_.end(), Vector3d());
  plane_orig_ = sum_pts / pts_.size();
}

boost::optional<Eigen::Vector3d> ConvexFaceFeature::snap(const Eigen::Vector3d &pt) const
{
  using namespace Eigen;
  //project point onto plane
  Vector3d pox = pt - plane_orig_;
  Vector3d pp = pt - pox.dot(plane_normal_)*plane_normal_;

  //for simplicity we'll only snap if the projected point is within the polygon defined by the face points
  if (pointInsideFace(pp))
  {
    Vector3d ppx = pt - pp;
    if (ppx.norm() <= dist_)
      return pp;
  }
  return boost::none;
}

visualization_msgs::Marker ConvexFaceFeature::visualization() const
{
  auto eigen_to_geometry_msgs = [](const Eigen::Vector3d& pt) -> geometry_msgs::Point
  {
    geometry_msgs::Point gpt;
    gpt.x = pt.x();
    gpt.y = pt.y();
    gpt.z = pt.z();
    return gpt;
  };

  const double ARROW_LEN = 0.1;

  visualization_msgs::Marker marker;
  marker.ns = "snap_feature_" + name_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.push_back(eigen_to_geometry_msgs(plane_orig_));
  marker.points.push_back(eigen_to_geometry_msgs(plane_orig_ + ARROW_LEN*plane_normal_));
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.0;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

bool ConvexFaceFeature::pointInsideFace(const Eigen::Vector3d &pt) const
{
  using namespace Eigen;
  //Iterate through each segment and check which side the point falls on
  std::vector<bool> pos_sides(pts_.size());
  for (int i=0; i < pts_.size(); ++i)
  {
    if (pts_[i] == pt)
      return true;

    Vector3d p1 = pts_[i];
    Vector3d p2 = (i == pts_.size()-1) ? pts_[0] : pts_[i+1];

    Vector3d vl = p2 - p1;
    Vector3d vp = pt - pts_[i];
    pos_sides[i] = vp.cross(vl).dot(plane_normal_) > 0;
  }

  if (std::all_of(pos_sides.begin(), pos_sides.end(), [](bool b) {return b;}))
    return true;
  if (std::none_of(pos_sides.begin(), pos_sides.end(), [](bool b) {return b;}))
    return true;
  return false;
}

}
