#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelToH12Loco : public rclcpp::Node {
 public:
  CmdVelToH12Loco() : Node("cmd_vel_to_h1_2_loco") {
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cmd_vel_nav");
    udp_host_ = declare_parameter<std::string>("udp_host", "127.0.0.1");
    if (udp_host_ != "127.0.0.1") {
      throw std::invalid_argument("udp_host must remain 127.0.0.1");
    }
    udp_port_ = declare_parameter<int>("udp_port", 39001);
    enabled_ = declare_parameter<bool>("enabled", false);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    declare_parameter<bool>("continuous_move", true);
    send_zero_when_idle_ = declare_parameter<bool>("send_zero_when_idle", true);
    declare_parameter<std::string>("network_interface", "");
    declare_parameter<int>("domain_id", 0);
    max_x_ = declare_parameter<double>("max_x", 0.10);
    max_y_ = declare_parameter<double>("max_y", 0.0);
    max_yaw_ = declare_parameter<double>("max_yaw", 0.30);
    timeout_ = declare_parameter<double>("timeout", 0.4);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    command_duration_ = declare_parameter<double>("command_duration", 0.5);

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      throw std::runtime_error("failed to create UDP socket");
    }
    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(udp_port_));
    if (inet_pton(AF_INET, udp_host_.c_str(), &addr_.sin_addr) != 1) {
      throw std::runtime_error("invalid UDP host: " + udp_host_);
    }

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          last_cmd_ = *msg;
          last_cmd_time_ = now();
          have_cmd_ = true;
        });

    if (enabled_ && auto_start_) {
      send_line("start");
    }

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0))),
        [this]() { publish_loco_command(); });

    RCLCPP_INFO(
        get_logger(),
        "H1-2 UDP bridge %s: %s -> %s:%d, max vx %.3f, max vy %.3f, max yaw %.3f",
        enabled_ ? "ENABLED" : "disabled", cmd_topic_.c_str(), udp_host_.c_str(),
        udp_port_, max_x_, max_y_, max_yaw_);
  }

  ~CmdVelToH12Loco() override {
    if (enabled_) {
      send_velocity(0.0, 0.0, 0.0);
    }
    if (sock_ >= 0) {
      close(sock_);
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

    send_velocity(vx, vy, wz);
  }

  void send_velocity(double vx, double vy, double wz) {
    send_line("vel " + std::to_string(static_cast<float>(vx)) + " " +
              std::to_string(static_cast<float>(vy)) + " " +
              std::to_string(static_cast<float>(wz)) + " " +
              std::to_string(static_cast<float>(command_duration_)));
  }

  void send_line(const std::string& line) {
    sendto(sock_, line.data(), line.size(), 0, reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
  }

  static double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
  }

  int sock_{-1};
  sockaddr_in addr_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool have_cmd_{false};
  bool enabled_{false};
  bool auto_start_{false};
  bool send_zero_when_idle_{true};
  std::string cmd_topic_;
  std::string udp_host_;
  int udp_port_{39001};
  double max_x_{0.10};
  double max_y_{0.0};
  double max_yaw_{0.30};
  double timeout_{0.4};
  double publish_rate_{10.0};
  double command_duration_{0.5};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToH12Loco>());
  rclcpp::shutdown();
  return 0;
}
