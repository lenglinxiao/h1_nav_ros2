#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#ifdef H1_HAVE_KISS_MATCHER
#include <kiss_matcher/KISSMatcher.hpp>
#endif

#ifdef H1_HAVE_SMALL_GICP
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#endif

#include "h1_2_icp_localization/msg/localization_status.hpp"
#include "h1_2_icp_localization/cloud_accumulator.hpp"
#include "h1_2_icp_localization/relocalization_state.hpp"
#include "h1_2_icp_localization/srv/relocalize.hpp"

namespace h1_2_icp_localization
{

namespace
{

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using FeatureT = pcl::FPFHSignature33;
using FeatureCloudT = pcl::PointCloud<FeatureT>;

#ifdef H1_HAVE_SMALL_GICP
using SmallGicpCloudT = small_gicp::PointCloud;
using SmallGicpCloudPtr = std::shared_ptr<SmallGicpCloudT>;
using SmallGicpTreeT = small_gicp::KdTree<SmallGicpCloudT>;
using SmallGicpRegistrationT =
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>;
#endif

Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  transform(0, 3) = pose.position.x;
  transform(1, 3) = pose.position.y;
  transform(2, 3) = pose.position.z;
  return transform;
}

geometry_msgs::msg::TransformStamped matrixToTransformMsg(
  const Eigen::Matrix4d & transform,
  const rclcpp::Time & stamp,
  const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped msg;
  Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
  q.normalize();
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = transform(0, 3);
  msg.transform.translation.y = transform(1, 3);
  msg.transform.translation.z = transform(2, 3);
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

geometry_msgs::msg::Pose matrixToPoseMsg(const Eigen::Matrix4d & transform)
{
  geometry_msgs::msg::Pose pose;
  Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
  q.normalize();
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

bool poseHasUsableOrientation(const geometry_msgs::msg::Pose & pose)
{
  const double norm =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  return std::isfinite(norm) && norm > 1e-9;
}

CloudT::Ptr downsample(const CloudT::Ptr & cloud, double leaf_size)
{
  CloudT::Ptr filtered(new CloudT);
  if (!cloud || cloud->empty()) {
    return filtered;
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(cloud);
  voxel.filter(*filtered);
  return filtered;
}

#ifdef H1_HAVE_KISS_MATCHER
std::vector<Eigen::Vector3f> toKissPoints(const CloudT::Ptr & cloud)
{
  std::vector<Eigen::Vector3f> points;
  if (!cloud) {
    return points;
  }
  points.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      points.emplace_back(point.x, point.y, point.z);
    }
  }
  return points;
}
#endif

#ifdef H1_HAVE_SMALL_GICP
SmallGicpCloudPtr toSmallGicpCloud(const CloudT::Ptr & cloud)
{
  auto converted = std::make_shared<SmallGicpCloudT>();
  if (!cloud) {
    return converted;
  }
  converted->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto & point = cloud->points[i];
    converted->point(i) << point.x, point.y, point.z, 1.0;
  }
  return converted;
}
#endif

FeatureCloudT::Ptr computeFeatures(
  const CloudT::Ptr & cloud,
  double normal_radius,
  double feature_radius,
  int num_threads)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setNumberOfThreads(num_threads);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setRadiusSearch(normal_radius);
  normal_estimator.compute(*normals);

  FeatureCloudT::Ptr features(new FeatureCloudT);
  pcl::FPFHEstimationOMP<PointT, pcl::Normal, FeatureT> feature_estimator;
  feature_estimator.setNumberOfThreads(num_threads);
  feature_estimator.setInputCloud(cloud);
  feature_estimator.setInputNormals(normals);
  feature_estimator.setRadiusSearch(feature_radius);
  feature_estimator.compute(*features);
  return features;
}

}  // namespace

class GlobalRelocalizationNode : public rclcpp::Node
{
public:
  GlobalRelocalizationNode()
  : Node("global_relocalization_node"),
    state_machine_(declare_parameter<int>("gicp_max_consecutive_failures", 3)),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    declareParameters();
    loadMap();
    setupSubscriptions();

    const auto registration_period =
      std::chrono::duration<double>(1.0 / std::max(registration_rate_hz_, 0.1));
    registration_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(registration_period),
      std::bind(&GlobalRelocalizationNode::onRegistrationTimer, this));

    const auto tf_period =
      std::chrono::duration<double>(1.0 / std::max(tf_rate_hz_, 1.0));
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tf_period),
      std::bind(&GlobalRelocalizationNode::onTfTimer, this));
  }

private:
  struct MapSnapshot
  {
    bool loaded = false;
    CloudT::Ptr global_map;
    CloudT::Ptr global_target;
    CloudT::Ptr tracking_target;
    FeatureCloudT::Ptr global_target_features;
#ifdef H1_HAVE_SMALL_GICP
    SmallGicpCloudPtr small_gicp_target;
    std::shared_ptr<SmallGicpTreeT> small_gicp_target_tree;
#endif
  };

  void declareParameters()
  {
    pcd_path_ = declare_parameter<std::string>("pcd_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "body");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/slam_odom");
    status_topic_ = declare_parameter<std::string>("status_topic", "/icp_localization/status");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    use_global_initialization_ = declare_parameter<bool>("use_global_initialization", true);
    use_global_recovery_ = declare_parameter<bool>("use_global_recovery", true);
    verify_global_with_gicp_ = declare_parameter<bool>("verify_global_with_gicp", true);
    use_kiss_matcher_ = declare_parameter<bool>("use_kiss_matcher", true);
    use_small_gicp_ = declare_parameter<bool>("use_small_gicp", true);
    global_voxel_resolution_ = declare_parameter<double>("global_voxel_resolution", 0.35);
    tracking_voxel_resolution_ = declare_parameter<double>("tracking_voxel_resolution", 0.20);
    normal_radius_ = declare_parameter<double>("normal_radius", 0.8);
    feature_radius_ = declare_parameter<double>("feature_radius", 1.5);
    gicp_max_correspondence_distance_ =
      declare_parameter<double>("gicp_max_correspondence_distance", 1.0);
    gicp_score_threshold_ = declare_parameter<double>("gicp_score_threshold", 0.35);
    small_gicp_max_dist_sq_ = declare_parameter<double>("small_gicp_max_dist_sq", 1.0);
    global_score_threshold_ = declare_parameter<double>("global_score_threshold", 0.5);
    kiss_min_final_inliers_ = declare_parameter<int>("kiss_min_final_inliers", 5);
    kiss_num_max_corr_ = declare_parameter<int>("kiss_num_max_corr", 5000);
    kiss_use_quatro_ = declare_parameter<bool>("kiss_use_quatro", false);
    kiss_use_ratio_test_ = declare_parameter<bool>("kiss_use_ratio_test", true);
    recovery_min_points_ = declare_parameter<int>("recovery_min_points", 1000);
    min_input_points_ = declare_parameter<int>("min_input_points", 100);
    max_accumulated_points_ = declare_parameter<int>("max_accumulated_points", 120000);
    recovery_cooldown_sec_ = declare_parameter<double>("recovery_cooldown_sec", 2.0);
    registration_rate_hz_ = declare_parameter<double>("registration_rate_hz", 2.0);
    tf_rate_hz_ = declare_parameter<double>("tf_rate_hz", 20.0);
    transform_tolerance_sec_ = declare_parameter<double>("transform_tolerance_sec", 0.5);
    num_threads_ = declare_parameter<int>("num_threads", 4);
    num_neighbors_ = declare_parameter<int>("num_neighbors", 20);
    sac_max_iterations_ = declare_parameter<int>("sac_max_iterations", 50000);
    sac_num_samples_ = declare_parameter<int>("sac_num_samples", 3);
    sac_correspondence_randomness_ = declare_parameter<int>("sac_correspondence_randomness", 5);
    sac_similarity_threshold_ = declare_parameter<double>("sac_similarity_threshold", 0.75);
    sac_max_correspondence_distance_ =
      declare_parameter<double>("sac_max_correspondence_distance", 1.5);
    sac_inlier_fraction_ = declare_parameter<double>("sac_inlier_fraction", 0.25);
    min_input_points_ = std::max(1, min_input_points_);
    kiss_min_final_inliers_ = std::max(1, kiss_min_final_inliers_);
    recovery_min_points_ = std::max(min_input_points_, recovery_min_points_);
    if (max_accumulated_points_ > 0) {
      max_accumulated_points_ = std::max(max_accumulated_points_, recovery_min_points_);
    }
  }

  void loadMap()
  {
    if (pcd_path_.empty()) {
      publishMapSnapshot(
        CloudT::Ptr(new CloudT), CloudT::Ptr(new CloudT), CloudT::Ptr(new CloudT),
        FeatureCloudT::Ptr(new FeatureCloudT),
#ifdef H1_HAVE_SMALL_GICP
        std::make_shared<SmallGicpCloudT>(), std::shared_ptr<SmallGicpTreeT>(),
#endif
        false, "PCD map path is empty");
      RCLCPP_WARN(get_logger(), "%s", last_message_.c_str());
      return;
    }

    CloudT::Ptr loaded_map(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(pcd_path_, *loaded_map) != 0) {
      publishMapSnapshot(
        CloudT::Ptr(new CloudT), CloudT::Ptr(new CloudT), CloudT::Ptr(new CloudT),
        FeatureCloudT::Ptr(new FeatureCloudT),
#ifdef H1_HAVE_SMALL_GICP
        std::make_shared<SmallGicpCloudT>(), std::shared_ptr<SmallGicpTreeT>(),
#endif
        false, "failed to load PCD map");
      RCLCPP_WARN(get_logger(), "%s: %s", last_message_.c_str(), pcd_path_.c_str());
      return;
    }

    CloudT::Ptr tracking_target = downsample(loaded_map, tracking_voxel_resolution_);
    CloudT::Ptr global_target = downsample(loaded_map, global_voxel_resolution_);
    FeatureCloudT::Ptr global_target_features(new FeatureCloudT);
    if (!global_target->empty()) {
      global_target_features =
        computeFeatures(global_target, normal_radius_, feature_radius_, num_threads_);
    }

#ifdef H1_HAVE_SMALL_GICP
    SmallGicpCloudPtr small_gicp_target;
    std::shared_ptr<SmallGicpTreeT> small_gicp_target_tree;
    if (use_small_gicp_ && !loaded_map->empty()) {
      small_gicp_target =
        small_gicp::voxelgrid_sampling_omp<SmallGicpCloudT, SmallGicpCloudT>(
          *toSmallGicpCloud(loaded_map), tracking_voxel_resolution_, num_threads_);
      if (small_gicp_target && !small_gicp_target->empty()) {
        small_gicp::estimate_covariances_omp(*small_gicp_target, num_neighbors_, num_threads_);
        small_gicp_target_tree = std::make_shared<SmallGicpTreeT>(
          small_gicp_target, small_gicp::KdTreeBuilderOMP(num_threads_));
      }
    }
#endif

    const bool loaded = !tracking_target->empty() && !global_target->empty();
    publishMapSnapshot(
      loaded_map,
      global_target,
      tracking_target,
      global_target_features,
#ifdef H1_HAVE_SMALL_GICP
      small_gicp_target,
      small_gicp_target_tree,
#endif
      loaded,
      loaded ? "PCD map loaded" : "PCD map is empty after downsampling");
    RCLCPP_INFO(
      get_logger(), "Loaded PCD map: raw=%zu, global=%zu, tracking=%zu",
      loaded_map->size(), global_target->size(), tracking_target->size());
  }

  void publishMapSnapshot(
    const CloudT::Ptr & global_map,
    const CloudT::Ptr & global_target,
    const CloudT::Ptr & tracking_target,
    const FeatureCloudT::Ptr & global_target_features,
#ifdef H1_HAVE_SMALL_GICP
    const SmallGicpCloudPtr & small_gicp_target,
    const std::shared_ptr<SmallGicpTreeT> & small_gicp_target_tree,
#endif
    bool loaded,
    const std::string & message)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    global_map_ = global_map;
    global_target_ = global_target;
    tracking_target_ = tracking_target;
    global_target_features_ = global_target_features;
#ifdef H1_HAVE_SMALL_GICP
    small_gicp_target_ = small_gicp_target;
    small_gicp_target_tree_ = small_gicp_target_tree;
#endif
    map_loaded_ = loaded;
    last_message_ = message;
  }

  MapSnapshot snapshotMap() const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    MapSnapshot snapshot;
    snapshot.loaded = map_loaded_;
    snapshot.global_map = global_map_;
    snapshot.global_target = global_target_;
    snapshot.tracking_target = tracking_target_;
    snapshot.global_target_features = global_target_features_;
#ifdef H1_HAVE_SMALL_GICP
    snapshot.small_gicp_target = small_gicp_target_;
    snapshot.small_gicp_target_tree = small_gicp_target_tree_;
#endif
    return snapshot;
  }

  bool isMapLoaded() const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return map_loaded_;
  }

  void setupSubscriptions()
  {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GlobalRelocalizationNode::onCloud, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50,
      std::bind(&GlobalRelocalizationNode::onOdom, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&GlobalRelocalizationNode::onInitialPose, this, std::placeholders::_1));
    relocalize_service_ = create_service<h1_2_icp_localization::srv::Relocalize>(
      "icp_relocalize",
      std::bind(&GlobalRelocalizationNode::onRelocalize, this, std::placeholders::_1, std::placeholders::_2));
    status_pub_ = create_publisher<msg::LocalizationStatus>(status_topic_, 10);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, finite_indices);
    if (static_cast<int>(cloud->size()) < min_input_points_) {
      last_message_ = "input cloud has too few finite points";
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_cloud_ = cloud;
    latest_cloud_stamp_ = msg->header.stamp;
    if (state_machine_.state() != RelocalizationState::TRACKING) {
      *accumulated_cloud_ += *cloud;
      if (max_accumulated_points_ > 0) {
        keepNewestPoints(*accumulated_cloud_, static_cast<std::size_t>(max_accumulated_points_));
      }
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_pose_ = poseMsgToMatrix(msg->pose.pose);
    latest_odom_stamp_ = msg->header.stamp;
    have_odom_ = true;
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_odom_) {
        last_message_ = "received initialpose but odometry is not available";
        publishStatus(false, last_score_, last_message_);
        return;
      }
      odom_to_base = latest_odom_pose_;
    }

    const Eigen::Matrix4d map_to_base = poseMsgToMatrix(msg->pose.pose);
    const Eigen::Matrix4d map_to_odom = computeMapToOdom(map_to_base, odom_to_base);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_to_odom_ = map_to_odom;
      localized_ = true;
      degraded_ = false;
      accumulated_cloud_->clear();
      state_machine_.onGlobalInitializationSucceeded();
    }
    last_message_ = "manual initial pose accepted";
    publishStatus(true, 0.0, last_message_);
  }

  void onRelocalize(
    const std::shared_ptr<h1_2_icp_localization::srv::Relocalize::Request> request,
    std::shared_ptr<h1_2_icp_localization::srv::Relocalize::Response> response)
  {
    const std::string requested_pcd = request->pcd_path.empty() ? pcd_path_ : request->pcd_path;
    if (!requested_pcd.empty() && requested_pcd != pcd_path_) {
      pcd_path_ = requested_pcd;
      loadMap();
    }

    if (!isMapLoaded()) {
      response->success = false;
      response->score = last_score_;
      response->aligned_pose = geometry_msgs::msg::Pose();
      response->message = last_message_;
      publishStatus(false, last_score_, last_message_);
      return;
    }

    if (poseHasUsableOrientation(request->initial_pose)) {
      Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!have_odom_) {
          response->success = false;
          response->score = last_score_;
          response->aligned_pose = geometry_msgs::msg::Pose();
          response->message = "received relocalize pose but odometry is not available";
          last_message_ = response->message;
          publishStatus(false, last_score_, last_message_);
          return;
        }
        odom_to_base = latest_odom_pose_;
      }

      const Eigen::Matrix4d map_to_base = poseMsgToMatrix(request->initial_pose);
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_to_odom_ = computeMapToOdom(map_to_base, odom_to_base);
        localized_ = true;
        degraded_ = false;
        accumulated_cloud_->clear();
        state_machine_.onGlobalInitializationSucceeded();
      }
      last_score_ = 0.0;
      last_message_ = "manual relocalize pose accepted";
      response->success = true;
      response->score = last_score_;
      response->aligned_pose = request->initial_pose;
      response->message = last_message_;
      publishStatus(true, last_score_, last_message_);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      accumulated_cloud_->clear();
      degraded_ = localized_;
      if (localized_) {
        state_machine_.forceGlobalRecovery();
      } else {
        state_machine_.resetToGlobalInitialization();
      }
      response->aligned_pose = localized_ ?
        matrixToPoseMsg(map_to_odom_ * latest_odom_pose_) :
        geometry_msgs::msg::Pose();
    }
    last_message_ = "global relocalization requested";
    response->success = true;
    response->score = last_score_;
    response->message = last_message_;
    publishStatus(false, last_score_, last_message_);
  }

  void onRegistrationTimer()
  {
    const MapSnapshot map = snapshotMap();
    if (!map.loaded) {
      publishStatus(false, last_score_, last_message_);
      return;
    }

    switch (state_machine_.state()) {
      case RelocalizationState::GLOBAL_INIT:
        runGlobalInitialization(map);
        return;
      case RelocalizationState::TRACKING:
        runTracking(map);
        return;
      case RelocalizationState::GLOBAL_RECOVERY:
        runGlobalRecovery(map);
        return;
    }
  }

  void runGlobalInitialization(const MapSnapshot & map)
  {
    if (!use_global_initialization_) {
      state_machine_.onGlobalInitializationSucceeded();
      return;
    }
    runGlobalAlignment(map, "global initialization");
  }

  void runGlobalRecovery(const MapSnapshot & map)
  {
    if (!use_global_recovery_) {
      degraded_ = true;
      publishStatus(false, last_score_, "global recovery disabled");
      return;
    }
    runGlobalAlignment(map, "global recovery");
  }

  void runGlobalAlignment(const MapSnapshot & map, const std::string & phase)
  {
    CloudT::Ptr source;
    Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_odom_) {
        publishStatus(false, last_score_, phase + " waiting for odometry");
        return;
      }
      if (static_cast<int>(accumulated_cloud_->size()) < recovery_min_points_) {
        publishStatus(false, last_score_, phase + " waiting for accumulated points");
        return;
      }
      const auto now = this->now();
      if (last_recovery_attempt_.nanoseconds() != 0 &&
        (now - last_recovery_attempt_).seconds() < recovery_cooldown_sec_)
      {
        return;
      }
      last_recovery_attempt_ = now;
      source.reset(new CloudT(*accumulated_cloud_));
      odom_to_base = latest_odom_pose_;
    }

    Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
    double score = 0.0;
    if (!tryGlobalAlignment(source, map, map_to_base, score)) {
      degraded_ = localized_;
      last_score_ = score;
      publishStatus(false, score, phase + " failed");
      return;
    }

    if (verify_global_with_gicp_) {
      Eigen::Matrix4d refined = Eigen::Matrix4d::Identity();
      double refined_score = 0.0;
      if (!tryGicp(source, map, map_to_base, refined, refined_score)) {
        degraded_ = localized_;
        last_score_ = refined_score;
        publishStatus(false, refined_score, phase + " GICP verification failed");
        return;
      }
      map_to_base = refined;
      score = refined_score;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_to_odom_ = computeMapToOdom(map_to_base, odom_to_base);
      localized_ = true;
      degraded_ = false;
      accumulated_cloud_->clear();
      if (state_machine_.state() == RelocalizationState::GLOBAL_INIT) {
        state_machine_.onGlobalInitializationSucceeded();
      } else {
        state_machine_.onGlobalRecoverySucceeded();
      }
    }
    last_score_ = score;
    publishStatus(true, score, successMessageForPhase(phase));
  }

  void runTracking(const MapSnapshot & map)
  {
    CloudT::Ptr source;
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!localized_ || !have_odom_ || !latest_cloud_ || latest_cloud_->empty()) {
        publishStatus(false, last_score_, "tracking waiting for localization, odometry, or cloud");
        return;
      }
      source = latest_cloud_;
      odom_to_base = latest_odom_pose_;
      initial_guess = map_to_odom_ * odom_to_base;
    }

    Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
    double score = 0.0;
    if (tryGicp(source, map, initial_guess, map_to_base, score)) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_to_odom_ = computeMapToOdom(map_to_base, odom_to_base);
      degraded_ = false;
      state_machine_.onTrackingSucceeded();
      last_score_ = score;
      publishStatus(true, score, tracking_success_message_);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      degraded_ = true;
      state_machine_.onTrackingFailed();
      if (state_machine_.state() == RelocalizationState::GLOBAL_RECOVERY) {
        accumulated_cloud_->clear();
      }
    }
    last_score_ = score;
    publishStatus(false, score, tracking_failure_message_);
  }

  std::string successMessageForPhase(const std::string & phase) const
  {
    if (last_global_alignment_backend_ == "kiss") {
      return phase == "global initialization" ?
        "KISSMatcher global initialization succeeded" :
        "KISSMatcher global recovery succeeded";
    }
    return phase + " succeeded";
  }

  bool tryGlobalAlignment(
    const CloudT::Ptr & source_cloud,
    const MapSnapshot & map,
    Eigen::Matrix4d & map_to_base,
    double & score)
  {
#ifdef H1_HAVE_KISS_MATCHER
    if (use_kiss_matcher_ && tryKissAlignment(source_cloud, map, map_to_base, score)) {
      last_global_alignment_backend_ = "kiss";
      return true;
    }
#endif
    last_global_alignment_backend_ = "pcl_sac";
    CloudT::Ptr source = downsample(source_cloud, global_voxel_resolution_);
    if (source->empty() || !map.global_target || map.global_target->empty() ||
      !map.global_target_features || map.global_target_features->empty())
    {
      score = std::numeric_limits<double>::infinity();
      return false;
    }

    FeatureCloudT::Ptr source_features =
      computeFeatures(source, normal_radius_, feature_radius_, num_threads_);
    pcl::SampleConsensusPrerejective<PointT, PointT, FeatureT> align;
    align.setInputSource(source);
    align.setSourceFeatures(source_features);
    align.setInputTarget(map.global_target);
    align.setTargetFeatures(map.global_target_features);
    align.setMaximumIterations(sac_max_iterations_);
    align.setNumberOfSamples(sac_num_samples_);
    align.setCorrespondenceRandomness(sac_correspondence_randomness_);
    align.setSimilarityThreshold(static_cast<float>(sac_similarity_threshold_));
    align.setMaxCorrespondenceDistance(static_cast<float>(sac_max_correspondence_distance_));
    align.setInlierFraction(static_cast<float>(sac_inlier_fraction_));

    CloudT aligned;
    align.align(aligned);
    score = align.getFitnessScore();
    if (!align.hasConverged() || !std::isfinite(score) || score > global_score_threshold_) {
      return false;
    }

    map_to_base = align.getFinalTransformation().cast<double>();
    return true;
  }

#ifdef H1_HAVE_KISS_MATCHER
  bool tryKissAlignment(
    const CloudT::Ptr & source_cloud,
    const MapSnapshot & map,
    Eigen::Matrix4d & map_to_base,
    double & score)
  {
    CloudT::Ptr source = downsample(source_cloud, global_voxel_resolution_);
    if (source->empty() || !map.global_target || map.global_target->empty()) {
      score = std::numeric_limits<double>::infinity();
      return false;
    }

    const auto source_points = toKissPoints(source);
    const auto target_points = toKissPoints(map.global_target);
    if (source_points.empty() || target_points.empty()) {
      score = std::numeric_limits<double>::infinity();
      return false;
    }

    kiss_matcher::KISSMatcherConfig config(
      static_cast<float>(global_voxel_resolution_),
      true,
      kiss_use_quatro_,
      1.0f,
      kiss_num_max_corr_);
    config.use_ratio_test_ = kiss_use_ratio_test_;
    kiss_matcher::KISSMatcher matcher(config);
    const kiss_matcher::RegistrationSolution solution =
      matcher.estimate(source_points, target_points);
    const size_t final_inliers = matcher.getNumFinalInliers();
    score = final_inliers > 0 ? 1.0 / static_cast<double>(final_inliers) :
      std::numeric_limits<double>::infinity();

    if (!solution.valid || final_inliers < static_cast<size_t>(kiss_min_final_inliers_)) {
      return false;
    }

    map_to_base = Eigen::Matrix4d::Identity();
    map_to_base.block<3, 3>(0, 0) = solution.rotation;
    map_to_base.block<3, 1>(0, 3) = solution.translation;
    return true;
  }
#endif

  bool tryGicp(
    const CloudT::Ptr & source_cloud,
    const MapSnapshot & map,
    const Eigen::Matrix4d & initial_guess,
    Eigen::Matrix4d & map_to_base,
    double & score)
  {
#ifdef H1_HAVE_SMALL_GICP
    if (use_small_gicp_ && trySmallGicp(source_cloud, map, initial_guess, map_to_base, score)) {
      tracking_success_message_ = "small_gicp tracking succeeded";
      tracking_failure_message_ = "small_gicp tracking failed";
      return true;
    }
#endif
    tracking_success_message_ = "GICP tracking succeeded";
    tracking_failure_message_ = "GICP tracking failed";
    CloudT::Ptr source = downsample(source_cloud, tracking_voxel_resolution_);
    if (source->empty() || !map.tracking_target || map.tracking_target->empty()) {
      score = std::numeric_limits<double>::infinity();
      return false;
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(map.tracking_target);
    gicp.setMaximumIterations(30);
    gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    CloudT aligned;
    gicp.align(aligned, initial_guess.cast<float>());
    score = gicp.getFitnessScore();
    if (!gicp.hasConverged() || !std::isfinite(score) || score > gicp_score_threshold_) {
      return false;
    }

    map_to_base = gicp.getFinalTransformation().cast<double>();
    return true;
  }

#ifdef H1_HAVE_SMALL_GICP
  bool trySmallGicp(
    const CloudT::Ptr & source_cloud,
    const MapSnapshot & map,
    const Eigen::Matrix4d & initial_guess,
    Eigen::Matrix4d & map_to_base,
    double & score)
  {
    if (!map.small_gicp_target || map.small_gicp_target->empty() || !map.small_gicp_target_tree) {
      score = std::numeric_limits<double>::infinity();
      return false;
    }

    SmallGicpCloudPtr source =
      small_gicp::voxelgrid_sampling_omp<SmallGicpCloudT, SmallGicpCloudT>(
        *toSmallGicpCloud(source_cloud), tracking_voxel_resolution_, num_threads_);
    if (!source || source->empty()) {
      score = std::numeric_limits<double>::infinity();
      return false;
    }

    small_gicp::estimate_covariances_omp(*source, num_neighbors_, num_threads_);

    SmallGicpRegistrationT registration;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq = small_gicp_max_dist_sq_;
    registration.optimizer.max_iterations = 30;

    const auto result = registration.align(*map.small_gicp_target, *source, *map.small_gicp_target_tree,
      Eigen::Isometry3d(initial_guess));

    score = result.num_inliers > 0 ?
      result.error / static_cast<double>(result.num_inliers) :
      std::numeric_limits<double>::infinity();
    if (!result.converged || !std::isfinite(score) || score > gicp_score_threshold_) {
      return false;
    }

    map_to_base = result.T_target_source.matrix();
    return true;
  }
#endif

  void onTfTimer()
  {
    if (!publish_tf_) {
      return;
    }
    Eigen::Matrix4d map_to_odom = Eigen::Matrix4d::Identity();
    bool should_publish = false;
    bool degraded = false;
    double score = std::numeric_limits<double>::infinity();
    rclcpp::Time stamp = now() + rclcpp::Duration::from_seconds(transform_tolerance_sec_);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      should_publish = localized_;
      map_to_odom = map_to_odom_;
      degraded = degraded_;
      score = last_score_;
    }
    if (should_publish) {
      tf_broadcaster_->sendTransform(
        matrixToTransformMsg(map_to_odom, stamp, map_frame_, odom_frame_));
      if (!tf_is_publishing_) {
        RCLCPP_INFO(
          get_logger(), "Publishing TF %s -> %s (score=%.3f, degraded=%s)",
          map_frame_.c_str(), odom_frame_.c_str(), score, degraded ? "true" : "false");
        tf_is_publishing_ = true;
      }
    } else if (tf_is_publishing_) {
      RCLCPP_WARN(
        get_logger(), "Stopped publishing TF %s -> %s because localization is unavailable.",
        map_frame_.c_str(), odom_frame_.c_str());
      tf_is_publishing_ = false;
    }
  }

  void publishStatus(bool converged, double score, const std::string & message)
  {
    msg::LocalizationStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.map_loaded = isMapLoaded();
    status.initialized = localized_;
    status.converged = converged;
    status.degraded = degraded_;
    status.score = score;
    status.message = message;
    status_pub_->publish(status);
  }

  std::string pcd_path_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string status_topic_;

  bool publish_tf_ = true;
  bool use_global_initialization_ = true;
  bool use_global_recovery_ = true;
  bool verify_global_with_gicp_ = true;
  bool use_kiss_matcher_ = true;
  bool use_small_gicp_ = true;
  bool kiss_use_quatro_ = false;
  bool kiss_use_ratio_test_ = true;
  bool map_loaded_ = false;
  bool localized_ = false;
  bool degraded_ = false;
  bool tf_is_publishing_ = false;
  bool have_odom_ = false;

  double global_voxel_resolution_ = 0.35;
  double tracking_voxel_resolution_ = 0.2;
  double normal_radius_ = 0.8;
  double feature_radius_ = 1.5;
  double gicp_max_correspondence_distance_ = 1.0;
  double gicp_score_threshold_ = 0.35;
  double small_gicp_max_dist_sq_ = 1.0;
  double global_score_threshold_ = 0.5;
  double recovery_cooldown_sec_ = 2.0;
  double registration_rate_hz_ = 2.0;
  double tf_rate_hz_ = 20.0;
  double transform_tolerance_sec_ = 0.5;
  double sac_similarity_threshold_ = 0.75;
  double sac_max_correspondence_distance_ = 1.5;
  double sac_inlier_fraction_ = 0.25;
  double last_score_ = 0.0;

  int recovery_min_points_ = 1000;
  int min_input_points_ = 100;
  int max_accumulated_points_ = 120000;
  int num_threads_ = 4;
  int num_neighbors_ = 20;
  int kiss_min_final_inliers_ = 5;
  int kiss_num_max_corr_ = 5000;
  int sac_max_iterations_ = 50000;
  int sac_num_samples_ = 3;
  int sac_correspondence_randomness_ = 5;

  std::string last_message_ = "waiting for global relocalization";
  std::string last_global_alignment_backend_ = "pcl_sac";
  std::string tracking_success_message_ = "GICP tracking succeeded";
  std::string tracking_failure_message_ = "GICP tracking failed";
  RelocalizationStateMachine state_machine_;

  std::mutex data_mutex_;
  mutable std::mutex map_mutex_;
  CloudT::Ptr global_map_;
  CloudT::Ptr global_target_;
  CloudT::Ptr tracking_target_;
  FeatureCloudT::Ptr global_target_features_;
#ifdef H1_HAVE_SMALL_GICP
  SmallGicpCloudPtr small_gicp_target_;
  std::shared_ptr<SmallGicpTreeT> small_gicp_target_tree_;
#endif
  CloudT::Ptr latest_cloud_;
  CloudT::Ptr accumulated_cloud_ = CloudT::Ptr(new CloudT);
  Eigen::Matrix4d latest_odom_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d map_to_odom_ = Eigen::Matrix4d::Identity();
  rclcpp::Time latest_cloud_stamp_;
  rclcpp::Time latest_odom_stamp_;
  rclcpp::Time last_recovery_attempt_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Service<h1_2_icp_localization::srv::Relocalize>::SharedPtr relocalize_service_;
  rclcpp::Publisher<msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace h1_2_icp_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<h1_2_icp_localization::GlobalRelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
