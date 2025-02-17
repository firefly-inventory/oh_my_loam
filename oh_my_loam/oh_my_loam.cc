#include "oh_my_loam.h"

#include <vector>
#include <pcl/io/pcd_io.h>

#include "common/pcl/pcl_utils.h"
#include "common/registerer/registerer.h"

namespace oh_my_loam {

namespace {
const double kPointMinDist = 0.5;
}  // namespace

bool OhMyLoam::Init() {
  config_ = common::YAMLConfig::Instance()->config();
  is_vis_ = config_["vis"].as<bool>();
  extractor_.reset(common::Registerer<Extractor>::NewInstance(
      "Extractor" + config_["lidar"].as<std::string>()));
  if (!extractor_->Init()) {
    AERROR << "Failed to initialize extractor";
    return false;
  }
  odometer_.reset(new Odometer);
  if (!odometer_->Init()) {
    AERROR << "Failed to initialize odometer";
    return false;
  }
  mapper_.reset(new Mapper);
  if (!mapper_->Init()) {
    AERROR << "Failed to initialize mapper";
    return false;
  }
  if (is_vis_) {
    visualizer_.reset(
        new OhmyloamVisualizer(config_["save_map_path"].as<std::string>()));
  }
  return true;
}

void OhMyLoam::Reset() {
  AWARN << "OhMySlam RESET";
  extractor_->Reset();
  odometer_->Reset();
  mapper_->Reset();
}

std::shared_ptr<OhmyloamVisFrame> OhMyLoam::Run(double timestamp,
                   const common::PointCloudConstPtr &cloud_in) {
  common::PointCloudPtr cloud(new common::PointCloud);
  RemoveOutliers(*cloud_in, cloud.get());
  std::vector<Feature> features;
  extractor_->Process(timestamp, cloud, &features);
  common::Pose3d pose_curr2odom;
  odometer_->Process(timestamp, features, &pose_curr2odom);
  common::Pose3d pose_curr2map;
  const auto &cloud_corn = odometer_->GetCloudCorn()->makeShared();
  const auto &cloud_surf = odometer_->GetCloudSurf()->makeShared();
  mapper_->Process(timestamp, cloud_corn, cloud_surf, pose_curr2odom,
                   &pose_curr2map);
  if (is_vis_) {
    Visualize(pose_curr2map, cloud_corn, cloud_surf, timestamp);
  }
  return GetFrame(pose_curr2map, cloud_corn, cloud_surf, timestamp);
}

std::shared_ptr<OhmyloamVisFrame> OhMyLoam::GetFrame(const common::Pose3d &pose_curr2map,
                         const TPointCloudConstPtr &cloud_corn,
                         const TPointCloudConstPtr &cloud_surf,
                         double timestamp) {
  std::shared_ptr<OhmyloamVisFrame> frame(new OhmyloamVisFrame);
  frame->timestamp = timestamp;
  frame->cloud_map_corn = mapper_->GetMapCloudCorn();
  frame->cloud_map_surf = mapper_->GetMapCloudSurf();
  frame->cloud_corn = cloud_corn;
  frame->cloud_surf = cloud_surf;
  frame->pose_map = pose_curr2map;
  return frame;
}

void OhMyLoam::Visualize(const common::Pose3d &pose_curr2map,
                         const TPointCloudConstPtr &cloud_corn,
                         const TPointCloudConstPtr &cloud_surf,
                         double timestamp) {
  auto frame = GetFrame(pose_curr2map, cloud_corn, cloud_surf, timestamp);
  visualizer_->Render(frame);
}

void OhMyLoam::RemoveOutliers(const common::PointCloud &cloud_in,
                              common::PointCloud *const cloud_out) const {
  common::RemovePoints<common::Point>(
      cloud_in, cloud_out, [&](const common::Point &pt) {
        return !common::IsFinite(pt) ||
               common::DistanceSquare(pt) < kPointMinDist * kPointMinDist;
      });
}

}  // namespace oh_my_loam