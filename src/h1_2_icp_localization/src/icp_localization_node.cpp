#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "h1_2_icp_localization/icp_localizer.hpp"
#include "h1_2_icp_localization/msg/localization_status.hpp"
#include "h1_2_icp_localization/srv/relocalize.hpp"

namespace h1_2_icp_localization
{

namespace
{

Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::Pose &pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  transform(0, 3) = pose.position.x;
  transform(1, 3) = pose.position.y;
  transform(2, 3) = pose.position.z;
  return transform;
}

geometry_msgs::msg::Pose matrixToPoseMsg(const Eigen::Matrix4d &transform)
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

geometry_msgs::msg::TransformStamped matrixToTransformMsg(
  const Eigen::Matrix4d &transform,
  const rclcpp::Time &stamp,
  const std::string &parent,
  const std::string &child)
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

}  // namespace

class IcpLocalizationNode : public rclcpp::Node
{
public:
  IcpLocalizationNode()
  : Node("icp_localization_node"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    pcd_path_ = declare_parameter<std::string>("pcd_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "body");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/slam_odom");
    status_topic_ = declare_parameter<std::string>("status_topic", "/icp_localization/status");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    const double refine_resolution = declare_parameter<double>("refine_resolution", 0.2);
    const double rough_resolution = declare_parameter<double>("rough_resolution", 0.5);
    const int refine_iter = declare_parameter<int>("refine_iter", 5);
    const int rough_iter = declare_parameter<int>("rough_iter", 10);
    score_threshold_ = declare_parameter<double>("score_threshold", 0.15);
    const double xy_offset = declare_parameter<double>("xy_offset", 1.0);
    const int xy_steps = declare_parameter<int>("xy_steps", 3);
    const int yaw_offset = declare_parameter<int>("yaw_offset", 1);
    const double yaw_resolution = declare_parameter<double>("yaw_resolution", 0.5);
    const double localize_rate_hz = declare_parameter<double>("localize_rate_hz", 1.0);

    localizer_.setParams(refine_resolution, rough_resolution, refine_iter, rough_iter, score_threshold_);
    localizer_.setSearchParams(xy_offset, xy_steps, yaw_offset, yaw_resolution);

    if (!pcd_path_.empty()) {
      map_loaded_ = localizer_.loadMap(pcd_path_);
      if (!map_loaded_) {
        RCLCPP_WARN(get_logger(), "Failed to load ICP PCD map: %s", pcd_path_.c_str());
      }
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&IcpLocalizationNode::onCloud, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      50,
      std::bind(&IcpLocalizationNode::onOdom, this, std::placeholders::_1));
    relocalize_service_ = create_service<h1_2_icp_localization::srv::Relocalize>(
      "icp_relocalize",
      std::bind(&IcpLocalizationNode::onRelocalize, this, std::placeholders::_1, std::placeholders::_2));
    status_pub_ = create_publisher<h1_2_icp_localization::msg::LocalizationStatus>(status_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(localize_rate_hz, 0.1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&IcpLocalizationNode::onTimer, this));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
    pcl::fromROSMsg(*msg, *cloud);
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_cloud_ = cloud;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_pose_ = poseMsgToMatrix(msg->pose.pose);
    latest_odom_stamp_ = msg->header.stamp;
    have_odom_ = true;
  }

  void onRelocalize(
    const std::shared_ptr<h1_2_icp_localization::srv::Relocalize::Request> request,
    std::shared_ptr<h1_2_icp_localization::srv::Relocalize::Response> response)
  {
    const std::string requested_pcd = request->pcd_path.empty() ? pcd_path_ : request->pcd_path;
    if (!requested_pcd.empty() && requested_pcd != pcd_path_) {
      pcd_path_ = requested_pcd;
      map_loaded_ = localizer_.loadMap(pcd_path_);
    }

    PointCloudXYZI::Ptr cloud;
    Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cloud = latest_cloud_;
      odom_pose = latest_odom_pose_;
    }

    const Eigen::Matrix4d initial_guess = poseMsgToMatrix(request->initial_pose);
    const IcpResult result = request->use_multi_align ?
      localizer_.multiAlign(cloud, initial_guess) :
      localizer_.align(cloud, initial_guess);

    response->success = result.success;
    response->score = result.score;
    response->aligned_pose = matrixToPoseMsg(result.pose);
    response->message = result.message;

    if (result.success) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_to_odom_ = result.pose * odom_pose.inverse();
      localized_ = true;
      degraded_ = false;
      last_success_stamp_ = now();
      last_message_ = result.message;
    } else {
      degraded_ = localized_;
      last_message_ = result.message;
    }
    publishStatus(result.success, result.score, result.message);
  }

  void onTimer()
  {
    PointCloudXYZI::Ptr cloud;
    Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d map_to_odom = Eigen::Matrix4d::Identity();
    bool can_track = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cloud = latest_cloud_;
      odom_pose = latest_odom_pose_;
      map_to_odom = map_to_odom_;
      can_track = localized_ && have_odom_;
    }

    if (map_loaded_ && can_track && cloud && !cloud->empty()) {
      const Eigen::Matrix4d initial_guess = map_to_odom * odom_pose;
      const IcpResult result = localizer_.align(cloud, initial_guess);
      if (result.success) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_to_odom_ = result.pose * odom_pose.inverse();
        degraded_ = false;
        last_success_stamp_ = now();
        last_message_ = result.message;
        map_to_odom = map_to_odom_;
      } else {
        degraded_ = true;
        last_message_ = result.message;
      }
      publishStatus(result.success, result.score, result.message);
    } else {
      publishStatus(false, localizer_.lastScore(), last_message_);
    }

    if (publish_tf_) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (localized_) {
        tf_broadcaster_->sendTransform(matrixToTransformMsg(map_to_odom_, now(), map_frame_, odom_frame_));
      }
    }
  }

  void publishStatus(bool converged, double score, const std::string &message)
  {
    h1_2_icp_localization::msg::LocalizationStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.map_loaded = map_loaded_;
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
  bool map_loaded_ = false;
  bool localized_ = false;
  bool degraded_ = false;
  bool have_odom_ = false;
  double score_threshold_ = 0.15;
  std::string last_message_ = "waiting for ICP relocalization";

  std::mutex data_mutex_;
  PointCloudXYZI::Ptr latest_cloud_;
  Eigen::Matrix4d latest_odom_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d map_to_odom_ = Eigen::Matrix4d::Identity();
  rclcpp::Time latest_odom_stamp_;
  rclcpp::Time last_success_stamp_;

  IcpLocalizer localizer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<h1_2_icp_localization::srv::Relocalize>::SharedPtr relocalize_service_;
  rclcpp::Publisher<h1_2_icp_localization::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace h1_2_icp_localization

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<h1_2_icp_localization::IcpLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
