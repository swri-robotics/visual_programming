/**
 * @file snap_features.h
 * @copyright (c) 2019 Southwest Research Institute
 */
#ifndef VPR_TRACKING_SNAP_FEATURE_H
#define VPR_TRACKING_SNAP_FEATURE_H

#include <memory>
#include <vector>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <xmlrpcpp/XmlRpc.h>
#include <visualization_msgs/Marker.h>

namespace vpr_tracking
{

enum class SnapType : int
{
  None = 0,
  Point,
  Line,
  Face
};

inline std::string to_string(const SnapType &sm)
{
  switch (sm)
  {
    case SnapType::None:
      return "None";
    case SnapType::Point:
      return "Point";
    case SnapType::Line:
      return "Line";
    case SnapType::Face:
      return "Face";
  }
}

class SnapableFeature
{
public:
  virtual SnapType type() const = 0;
  virtual std::string name() const = 0;
  virtual boost::optional<Eigen::Vector3d> snap(const Eigen::Vector3d &pt) const = 0;
  virtual visualization_msgs::Marker visualization() const = 0;
};

std::unique_ptr<SnapableFeature> snapFeatureFromParam(XmlRpc::XmlRpcValue &param);

class PointFeature : public SnapableFeature
{
public:
  PointFeature(const std::string &name, const Eigen::Vector3d &center, double dist) :
    name_(name),
    center_(center),
    dist_(dist)
  {}

  SnapType type() const {return SnapType::Point;}
  std::string name() const {return name_;}
  boost::optional<Eigen::Vector3d> snap(const Eigen::Vector3d &pt) const;
  visualization_msgs::Marker visualization() const;

  std::string name_;
  Eigen::Vector3d center_;
  double dist_;
};

class LineFeature : public SnapableFeature
{
public:
  LineFeature(const std::string &name, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, double dist) :
    name_(name),
    p1_(p1),
    p2_(p2),
    dist_(dist)
  {}

  SnapType type() const {return SnapType::Line;}
  std::string name() const {return name_;}
  boost::optional<Eigen::Vector3d> snap(const Eigen::Vector3d &pt) const;
  visualization_msgs::Marker visualization() const;

  std::string name_;
  Eigen::Vector3d p1_;
  Eigen::Vector3d p2_;
  double dist_;
};

class ConvexFaceFeature : public SnapableFeature
{
public:
  ConvexFaceFeature(const std::string &name, const std::vector<Eigen::Vector3d> &points, double dist);

  SnapType type() const {return SnapType::Face;}
  std::string name() const {return name_;}
  boost::optional<Eigen::Vector3d> snap(const Eigen::Vector3d &pt) const;
  visualization_msgs::Marker visualization() const;

  std::string name_;
  std::vector<Eigen::Vector3d> pts_;
  double dist_;

private:
  bool pointInsideFace(const Eigen::Vector3d &pt) const;

  Eigen::Vector3d plane_orig_;
  Eigen::Vector3d plane_normal_;
};

}

#endif
