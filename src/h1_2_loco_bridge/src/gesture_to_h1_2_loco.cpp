#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "h1_2_nav_msgs/srv/execute_gesture.hpp"
#include "rclcpp/rclcpp.hpp"

class GestureToH12Loco : public rclcpp::Node {
 public:
  GestureToH12Loco() : Node("gesture_to_h1_2_loco") {
    udp_host_ = declare_parameter<std::string>("udp_host", "127.0.0.1");
    udp_port_ = declare_parameter<int>("udp_port", 39001);
    service_name_ = declare_parameter<std::string>("service_name", "/h1_2_loco/execute_gesture");
    timeout_ = declare_parameter<double>("timeout", 20.0);
    shake_hand_duration_ = declare_parameter<double>("shake_hand_duration", 10.0);

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

    service_ = create_service<h1_2_nav_msgs::srv::ExecuteGesture>(
        service_name_,
        [this](const std::shared_ptr<h1_2_nav_msgs::srv::ExecuteGesture::Request> request,
               std::shared_ptr<h1_2_nav_msgs::srv::ExecuteGesture::Response> response) {
          handle_request(request, response);
        });

    RCLCPP_INFO(get_logger(), "H1-2 gesture bridge ready: %s -> %s:%d",
                service_name_.c_str(), udp_host_.c_str(), udp_port_);
  }

  ~GestureToH12Loco() override {
    if (sock_ >= 0) {
      close(sock_);
    }
  }

 private:
  void handle_request(
      const std::shared_ptr<h1_2_nav_msgs::srv::ExecuteGesture::Request>& request,
      const std::shared_ptr<h1_2_nav_msgs::srv::ExecuteGesture::Response>& response) {
    const std::string name = normalize_name(request->name);
    if (name.empty()) {
      response->success = false;
      response->sdk_ret = -2;
      response->message = "Gesture name is empty.";
      return;
    }

    const std::string line = build_command(name);
    sendto(sock_, line.data(), line.size(), 0, reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));

    char buf[256];
    timeval socket_timeout{};
    socket_timeout.tv_sec = static_cast<int>(timeout_);
    socket_timeout.tv_usec = static_cast<int>((timeout_ - socket_timeout.tv_sec) * 1000000.0);
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &socket_timeout, sizeof(socket_timeout));

    const ssize_t n = recv(sock_, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
      response->success = false;
      response->sdk_ret = -1;
      response->message = "Timed out waiting for H1-2 gesture daemon response.";
      return;
    }
    buf[n] = '\0';
    parse_daemon_response(std::string(buf), response);
  }

  std::string normalize_name(std::string name) const {
    const auto first = name.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = name.find_last_not_of(" \t\r\n");
    name = name.substr(first, last - first + 1);
    return name;
  }

  std::string build_command(const std::string& name) const {
    if (name == "shake_hand") {
      return "gesture " + name + " " + std::to_string(static_cast<float>(shake_hand_duration_));
    }
    return "gesture " + name;
  }

  void parse_daemon_response(
      const std::string& line,
      const std::shared_ptr<h1_2_nav_msgs::srv::ExecuteGesture::Response>& response) const {
    std::istringstream in(line);
    std::string status;
    int sdk_ret = -1;
    in >> status >> sdk_ret;
    std::string message;
    std::getline(in, message);
    if (!message.empty() && message.front() == ' ') {
      message.erase(0, 1);
    }
    response->sdk_ret = sdk_ret;
    response->success = (status == "ok" && sdk_ret == 0);
    response->message = message.empty() ? line : message;
  }

  int sock_{-1};
  sockaddr_in addr_{};
  rclcpp::Service<h1_2_nav_msgs::srv::ExecuteGesture>::SharedPtr service_;
  std::string udp_host_;
  std::string service_name_;
  int udp_port_{39001};
  double timeout_{20.0};
  double shake_hand_duration_{10.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GestureToH12Loco>());
  rclcpp::shutdown();
  return 0;
}
