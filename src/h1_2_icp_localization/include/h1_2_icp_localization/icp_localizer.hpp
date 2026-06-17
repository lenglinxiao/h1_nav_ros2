#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace h1_2_icp_localization
{

using PointXYZI = pcl::PointXYZI;
using PointNormal = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointXYZI>;
using PointCloudNormal = pcl::PointCloud<PointNormal>;

struct IcpResult
{
  bool success = false;
  double score = 0.0;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  std::string message;
};

class IcpLocalizer
{
public:
  IcpLocalizer();

  void setParams(double refine_resolution, double rough_resolution, int refine_iter, int rough_iter, double score_threshold);
  void setSearchParams(double xy_offset, int xy_steps, int yaw_offset, double yaw_resolution);

  bool loadMap(const std::string &pcd_path, bool with_normals = false);
  bool isInitialized() const { return initialized_; }
  double lastScore() const { return last_score_; }

  IcpResult align(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &initial_guess);
  IcpResult multiAlign(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &initial_guess);

private:
  PointCloudNormal::Ptr addNormals(PointCloudXYZI::Ptr cloud) const;
  IcpResult refineFromGuess(PointCloudXYZI::Ptr refine_source, const Eigen::Matrix4f &rough_transform);

  std::string pcd_path_;
  PointCloudNormal::Ptr rough_map_;
  PointCloudNormal::Ptr refine_map_;

  double refine_resolution_ = 0.2;
  double rough_resolution_ = 0.5;
  int refine_iter_ = 5;
  int rough_iter_ = 10;
  double score_threshold_ = 0.15;
  double last_score_ = 0.0;

  double xy_offset_ = 1.0;
  int xy_steps_ = 3;
  int yaw_offset_ = 1;
  double yaw_resolution_ = 0.5;
  bool initialized_ = false;

  pcl::VoxelGrid<PointXYZI> voxel_rough_filter_;
  pcl::VoxelGrid<PointXYZI> voxel_refine_filter_;
  pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> icp_rough_;
  pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> icp_refine_;
};

}  // namespace h1_2_icp_localization
