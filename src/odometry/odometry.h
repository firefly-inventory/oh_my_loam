#pragma once

#include <extractor/feature_points.h>

#include "common.h"
#include "helper/helper.h"

namespace oh_my_loam {

// frame to frame lidar odometry
class Odometry {
 public:
  Odometry() = default;

  bool Init(const YAML::Node& config);

  void Process(const FeaturePoints& feature, Pose3D* const pose);

 protected:
  void UpdatePre(const FeaturePoints& feature);
  void MatchCornPoints(const TPointCloud& src, const TPointCloud& tgt,
                       std::vector<PointLinePair>* const pairs,
                       double dist_thresh) const;

  void MatchSurfPoints(const TPointCloud& src, const TPointCloud& tgt,
                       std::vector<PointPlanePair>* const pairs,
                       double dist_thresh) const;

  Pose3D pose_curr2world_;
  Pose3D pose_curr2last_;

  TPointCloudPtr surf_pts_pre_;
  TPointCloudPtr corn_pts_pre_;

  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_surf_pts_;
  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_corn_pts_;

  bool is_initialized = false;
  YAML::Node config_;

  DISALLOW_COPY_AND_ASSIGN(Odometry)
};

}  // namespace oh_my_loam
