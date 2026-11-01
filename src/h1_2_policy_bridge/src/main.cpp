#include "h1_2_policy_bridge/node.hpp"
#include <chrono>
#include <cstdlib>
#include <cstring>
int main(int argc, char** argv) {
  const char* rmw = std::getenv("RMW_IMPLEMENTATION");
  if (rmw && std::strcmp(rmw, "rmw_fastrtps_cpp") != 0) {
    fprintf(stderr, "requires RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"); return 1;
  }
  if (!rmw) setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
  rclcpp::init(argc, argv);
  try {
    auto now = [] { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
    rclcpp::spin(std::make_shared<h1_policy::PolicyNode>(rclcpp::NodeOptions{}, h1_policy::make_sdk_io(), now));
  } catch (const std::exception& e) {
    fprintf(stderr, "%s\n", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
}
