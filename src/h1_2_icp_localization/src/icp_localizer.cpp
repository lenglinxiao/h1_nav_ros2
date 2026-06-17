#include "h1_2_icp_localization/icp_localizer.hpp"

#include <cmath>
#include <limits>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

namespace h1_2_icp_localization
{

namespace
{

Eigen::Vector3d rotationToRpy(const Eigen::Matrix3d &rotation)
{
  return rotation.eulerAngles(0, 1, 2);
}

}  // namespace

IcpLocalizer::IcpLocalizer()
{
  setParams(refine_resolution_, rough_resolution_, refine_iter_, rough_iter_, score_threshold_);
  setSearchParams(xy_offset_, xy_steps_, yaw_offset_, yaw_resolution_);
}

void IcpLocalizer::setParams(
  double refine_resolution,
  double rough_resolution,
  int refine_iter,
  int rough_iter,
  double score_threshold)
{
  refine_resolution_ = refine_resolution;
  rough_resolution_ = rough_resolution;
  refine_iter_ = refine_iter;
  rough_iter_ = rough_iter;
  score_threshold_ = score_threshold;

  voxel_rough_filter_.setLeafSize(rough_resolution_, rough_resolution_, rough_resolution_);
  voxel_refine_filter_.setLeafSize(refine_resolution_, refine_resolution_, refine_resolution_);

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setMaxCorrespondenceDistance(rough_resolution_ * 8.0);
  icp_rough_.setTransformationEpsilon(1e-6);
  icp_rough_.setEuclideanFitnessEpsilon(1e-6);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setMaxCorrespondenceDistance(refine_resolution_ * 8.0);
  icp_refine_.setTransformationEpsilon(1e-7);
  icp_refine_.setEuclideanFitnessEpsilon(1e-7);
}

void IcpLocalizer::setSearchParams(double xy_offset, int xy_steps, int yaw_offset, double yaw_resolution)
{
  xy_offset_ = xy_offset;
  xy_steps_ = xy_steps;
  yaw_offset_ = yaw_offset;
  yaw_resolution_ = yaw_resolution;
}

bool IcpLocalizer::loadMap(const std::string &pcd_path, bool with_normals)
{
  if (initialized_ && pcd_path_ == pcd_path) {
    return true;
  }

  if (with_normals) {
    refine_map_.reset(new PointCloudNormal);
    if (pcl::io::loadPCDFile<PointNormal>(pcd_path, *refine_map_) != 0) {
      initialized_ = false;
      return false;
    }
  } else {
    PointCloudXYZI::Ptr raw_map(new PointCloudXYZI);
    if (pcl::io::loadPCDFile<PointXYZI>(pcd_path, *raw_map) != 0) {
      initialized_ = false;
      return false;
    }
    PointCloudXYZI::Ptr refine_xyz(new PointCloudXYZI);
    voxel_refine_filter_.setInputCloud(raw_map);
    voxel_refine_filter_.filter(*refine_xyz);
    refine_map_ = addNormals(refine_xyz);
  }

  PointCloudXYZI::Ptr rough_xyz(new PointCloudXYZI);
  PointCloudXYZI::Ptr refine_xyz(new PointCloudXYZI);
  pcl::copyPointCloud(*refine_map_, *refine_xyz);
  voxel_rough_filter_.setInputCloud(refine_xyz);
  voxel_rough_filter_.filter(*rough_xyz);
  rough_map_ = addNormals(rough_xyz);

  icp_rough_.setInputTarget(rough_map_);
  icp_refine_.setInputTarget(refine_map_);

  pcd_path_ = pcd_path;
  initialized_ = true;
  return true;
}

PointCloudNormal::Ptr IcpLocalizer::addNormals(PointCloudXYZI::Ptr cloud) const
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointXYZI>::Ptr search_tree(new pcl::search::KdTree<PointXYZI>);
  search_tree->setInputCloud(cloud);

  pcl::NormalEstimation<PointXYZI, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setSearchMethod(search_tree);
  normal_estimator.setKSearch(15);
  normal_estimator.compute(*normals);

  PointCloudNormal::Ptr out(new PointCloudNormal);
  pcl::concatenateFields(*cloud, *normals, *out);
  return out;
}

IcpResult IcpLocalizer::refineFromGuess(PointCloudXYZI::Ptr refine_source, const Eigen::Matrix4f &rough_transform)
{
  IcpResult result;
  PointCloudNormal::Ptr refine_source_norm = addNormals(refine_source);
  PointCloudNormal::Ptr aligned(new PointCloudNormal);

  icp_refine_.setInputSource(refine_source_norm);
  icp_refine_.align(*aligned, rough_transform);
  result.score = icp_refine_.getFitnessScore();
  last_score_ = result.score;

  if (!icp_refine_.hasConverged()) {
    result.message = "refine ICP did not converge";
    return result;
  }
  if (result.score > score_threshold_) {
    result.message = "refine ICP score is above threshold";
    return result;
  }

  result.success = true;
  result.pose = icp_refine_.getFinalTransformation().cast<double>();
  result.message = "ICP alignment converged";
  return result;
}

IcpResult IcpLocalizer::align(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &initial_guess)
{
  IcpResult result;
  if (!initialized_) {
    result.message = "PCD map is not loaded";
    return result;
  }
  if (!source || source->empty()) {
    result.message = "source cloud is empty";
    return result;
  }

  PointCloudXYZI::Ptr rough_source(new PointCloudXYZI);
  PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);
  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  PointCloudNormal::Ptr rough_source_norm = addNormals(rough_source);
  PointCloudNormal::Ptr aligned(new PointCloudNormal);

  icp_rough_.setInputSource(rough_source_norm);
  icp_rough_.align(*aligned, initial_guess.cast<float>());
  last_score_ = icp_rough_.getFitnessScore();

  if (!icp_rough_.hasConverged()) {
    result.score = last_score_;
    result.message = "rough ICP did not converge";
    return result;
  }

  return refineFromGuess(refine_source, icp_rough_.getFinalTransformation());
}

IcpResult IcpLocalizer::multiAlign(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &initial_guess)
{
  IcpResult result;
  if (!initialized_) {
    result.message = "PCD map is not loaded";
    return result;
  }
  if (!source || source->empty()) {
    result.message = "source cloud is empty";
    return result;
  }

  PointCloudXYZI::Ptr rough_source(new PointCloudXYZI);
  PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);
  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  PointCloudNormal::Ptr rough_source_norm = addNormals(rough_source);
  PointCloudNormal::Ptr aligned(new PointCloudNormal);

  Eigen::Vector3d xyz = initial_guess.block<3, 1>(0, 3);
  Eigen::Vector3d rpy = rotationToRpy(initial_guess.block<3, 3>(0, 0));
  Eigen::AngleAxisf roll_angle(static_cast<float>(rpy.x()), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(rpy.y()), Eigen::Vector3f::UnitY());

  bool found_rough = false;
  double best_score = std::numeric_limits<double>::max();
  Eigen::Matrix4f best_rough_transform = Eigen::Matrix4f::Identity();

  for (int ix = -xy_steps_; ix <= xy_steps_; ++ix) {
    for (int iy = -xy_steps_; iy <= xy_steps_; ++iy) {
      for (int iz = -yaw_offset_; iz <= yaw_offset_; ++iz) {
        Eigen::AngleAxisf yaw_angle(
          static_cast<float>(rpy.z() + iz * yaw_resolution_),
          Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f candidate = Eigen::Matrix4f::Identity();
        candidate.block<3, 3>(0, 0) = (roll_angle * pitch_angle * yaw_angle).toRotationMatrix();
        candidate(0, 3) = static_cast<float>(xyz.x() + ix * xy_offset_);
        candidate(1, 3) = static_cast<float>(xyz.y() + iy * xy_offset_);
        candidate(2, 3) = static_cast<float>(xyz.z());

        icp_rough_.setInputSource(rough_source_norm);
        icp_rough_.align(*aligned, candidate);
        if (!icp_rough_.hasConverged()) {
          continue;
        }
        const double rough_score = icp_rough_.getFitnessScore();
        if (rough_score > score_threshold_ * 2.0) {
          continue;
        }
        if (rough_score < best_score) {
          found_rough = true;
          best_score = rough_score;
          best_rough_transform = icp_rough_.getFinalTransformation();
        }
      }
    }
  }

  if (!found_rough) {
    result.score = best_score;
    result.message = "no valid rough ICP candidate";
    return result;
  }

  return refineFromGuess(refine_source, best_rough_transform);
}

}  // namespace h1_2_icp_localization
