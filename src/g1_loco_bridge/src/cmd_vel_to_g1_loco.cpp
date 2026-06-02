#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "g1/g1_loco_client.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CmdVelToG1Loco : public rclcpp::Node {
 public:
  CmdVelToG1Loco() : Node("cmd_vel_to_g1_loco"), client_(this) {
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cmd_vel_nav");
    enabled_ = declare_parameter<bool>("enabled", false);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    continuous_move_ = declare_parameter<bool>("continuous_move", true);
    max_x_ = declare_parameter<double>("max_x", 0.15);
    max_y_ = declare_parameter<double>("max_y", 0.0);
    max_yaw_ = declare_parameter<double>("max_yaw", 0.35);
    timeout_ = declare_parameter<double>("timeout", 0.4);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    command_duration_ = declare_parameter<double>("command_duration", 0.5);
    send_zero_when_idle_ = declare_parameter<bool>("send_zero_when_idle", true);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          last_cmd_ = *msg;
          last_cmd_time_ = now();
          have_cmd_ = true;
        });

    if (auto_start_) {
      const auto ret = client_.Start();
      report_ret("Start", ret);
    }
    client_.SwitchMoveMode(continuous_move_);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0))),
        [this]() { publish_loco_command(); });

    RCLCPP_INFO(
        get_logger(),
        "G1 loco bridge %s: %s -> /api/sport/request, max vx %.3f, max yaw %.3f",
        enabled_ ? "ENABLED" : "disabled", cmd_topic_.c_str(), max_x_, max_yaw_);
  }

  ~CmdVelToG1Loco() override {
    if (enabled_) {
      client_.StopMove();
    }
  }

 private:
  void publish_loco_command() {
    if (!enabled_) {
      return;
    }

    const auto age = have_cmd_ ? (now() - last_cmd_time_).seconds() : timeout_ + 1.0;
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    if (have_cmd_ && age <= timeout_) {
      vx = clamp(last_cmd_.linear.x, -max_x_, max_x_);
      vy = clamp(last_cmd_.linear.y, -max_y_, max_y_);
      wz = clamp(last_cmd_.angular.z, -max_yaw_, max_yaw_);
    } else if (!send_zero_when_idle_) {
      return;
    }

    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      vx = 0.0;
      vy = 0.0;
      wz = 0.0;
    }

    const auto ret = client_.SetVelocity(
        static_cast<float>(vx),
        static_cast<float>(vy),
        static_cast<float>(wz),
        static_cast<float>(command_duration_));

    if (ret != 0 && ret != last_error_) {
      report_ret("SetVelocity", ret);
      last_error_ = ret;
    } else if (ret == 0) {
      last_error_ = 0;
    }
  }

  void report_ret(const char* action, int32_t ret) {
    if (ret == 0) {
      RCLCPP_INFO(get_logger(), "%s OK", action);
    } else {
      RCLCPP_ERROR(get_logger(), "%s failed, code: %d", action, ret);
    }
  }

  static double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
  }

  unitree::robot::g1::LocoClient client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool have_cmd_{false};
  bool enabled_{false};
  bool auto_start_{false};
  bool continuous_move_{true};
  bool send_zero_when_idle_{true};
  std::string cmd_topic_;
  double max_x_{0.15};
  double max_y_{0.0};
  double max_yaw_{0.35};
  double timeout_{0.4};
  double publish_rate_{10.0};
  double command_duration_{0.5};
  int32_t last_error_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToG1Loco>());
  rclcpp::shutdown();
  return 0;
}
