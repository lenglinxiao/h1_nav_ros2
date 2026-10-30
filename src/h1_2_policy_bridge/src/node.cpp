#include "h1_2_policy_bridge/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
using namespace std::chrono_literals;
using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
class PolicyNode : public rclcpp::Node {
public:
PolicyNode() : Node("h1_2_policy_node"), controller_(declare_parameter<std::string>("model_path", "/home/llx/policy.onnx")) {
  auto interface = declare_parameter<std::string>("network_interface", "lo");
  auto domain = declare_parameter<int>("dds_domain", 42);
  if(interface != "lo"||domain != 42) throw std::runtime_error("phase 1 requires lo and DDS domain 42");
  auto topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_final");
  unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
  sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LS> >("rt/lowstate");
  sub_->InitChannel([this](const void* p){
      std::lock_guard<std::mutex> l(mutex_);
      state_msg_ = *static_cast<const LS*>(p);
      received_ = std::chrono::steady_clock::now();
      has_state_ = true;
    });
  pub_ = std::make_shared<unitree::robot::ChannelPublisher<LC> >("rt/lowcmd");
  pub_->InitChannel();
  velocity_ = create_subscription<geometry_msgs::msg::Twist>(topic, 10, [this](geometry_msgs::msg::Twist::SharedPtr m){
      cmd_ = {float(m->linear.x), float(m->linear.y), float(m->angular.z)};
      cmd_at_ = std::chrono::steady_clock::now();
    });
  for(auto item: {std::pair<const char*, h1_policy::Mode>{"passive", h1_policy::Mode::Passive}, {"fix_stand", h1_policy::Mode::FixStand}, {"arm_hold_stand", h1_policy::Mode::ArmHoldStand}, {"arm_hold", h1_policy::Mode::ArmHold}}) {
    services_.push_back(create_service<std_srvs::srv::Trigger>(std::string("~/") + item.first, [this, mode = item.second](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        h1_policy::State s;
        uint8_t mm = 0, pr = 0;
        if(!read(s, mm, pr)) {
          res->message = "fresh valid low state required";
          return;
        }
        res->success = controller_.transition(mode, s, now_seconds());
        if(res->success) {
          active_ = true;
          res->message = "accepted";
        }else res->message = "invalid transition or readiness conditions";
      }));
  }
  timer_ = create_wall_timer(20ms, [this] {
      tick();
    });
}
private:
static double now_seconds(){
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool read(
  h1_policy::State& s, uint8_t& machine, uint8_t& pr
  ){
  LS msg;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if(!has_state_||std::chrono::steady_clock::now() - received_ > 100ms) return false;
    msg = state_msg_;
  } machine = msg.mode_machine();
  pr = msg.mode_pr();
  if(machine != 0 || pr != 0) return false;
  h1_policy::Joints q{}, dq{};
  for(int i = 0; i < 27; i++) {
    q[i] = msg.motor_state()[i].q();
    dq[i] = msg.motor_state()[i].dq();
  } s.q = h1_policy::from_sdk(q);
  s.dq = h1_policy::from_sdk(dq);
  for(int i = 0; i < 3; i++) s.gyro[i] = msg.imu_state().gyroscope()[i];
  auto quat = msg.imu_state().quaternion();
  Eigen::Quaternionf rot(quat[0], quat[1], quat[2], quat[3]);
  if(!std::isfinite(rot.norm())||std::abs(rot.norm() - 1) > 0.05) return false;
  auto g = rot.conjugate() * Eigen::Vector3f(0, 0, -1);
  for(int i = 0; i < 3; i++) s.gravity[i] = g[i];
  return true;
}
void tick(){
  if(!active_) return;
  h1_policy::State s;
  uint8_t machine = 0, pr = 0;
  if(!read(s, machine, pr)) {
    controller_.fault();
    active_ = false;
    RCLCPP_ERROR(get_logger(), "LowState stale or invalid; stopped, manual recovery required");
    return;
  }
  auto c = cmd_;
  if(std::chrono::steady_clock::now() - cmd_at_ > 400ms) c = {0, 0, 0};
  for(auto x:c) if(!std::isfinite(x)) {
      controller_.fault();
      active_ = false;
      return;
    }
  c[0] = std::clamp(c[0], -.2f, .2f);
  c[1] = 0;
  c[2] = std::clamp(c[2], -.5f, .5f);
  try{
    auto t = h1_policy::to_sdk(controller_.step(s, c, now_seconds()));
    for (float torque : t) {
      if (!std::isfinite(torque)) throw std::runtime_error("non-finite torque");
    }
    LC out{};
    out.mode_machine() = machine;
    out.mode_pr() = pr;
    for(int i = 0; i < 27; i++) {
      auto& m = out.motor_cmd()[i];
      m.mode() = 1;
      m.tau() = t[i];
      m.kp() = 0;
      m.kd() = 0;
    } out.crc() = crc32_core(reinterpret_cast<uint32_t*>(&out), (sizeof(out) >> 2) - 1);
    if (!pub_->Write(out, 0)) throw std::runtime_error("LowCmd write failed");
  }catch(const std::exception&e) {
    controller_.fault();
    active_ = false;
    RCLCPP_ERROR(get_logger(), "Policy fault: %s", e.what());
  }
}
h1_policy::Controller controller_;
bool active_ = false, has_state_ = false;
std::mutex mutex_;
LS state_msg_{};
std::chrono::steady_clock::time_point received_{}, cmd_at_{};
h1_policy::Command cmd_{};
std::shared_ptr<unitree::robot::ChannelSubscriber<LS> > sub_;
std::shared_ptr<unitree::robot::ChannelPublisher<LC> > pub_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_;
std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
rclcpp::TimerBase::SharedPtr timer_;
};
int main(
  int argc, char**argv
  ){
  const char* rmw = std::getenv("RMW_IMPLEMENTATION");
  if (rmw && std::strcmp(rmw, "rmw_fastrtps_cpp") != 0) {
    fprintf(stderr, "phase 1 requires RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n");
    return 1;
  }
  if (!rmw) setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
  // Keep the advisory lock for the entire process lifetime; a stale file is harmless.
  const std::string lock_path = "/tmp/h1_2_policy_bridge_" + std::to_string(getuid()) + "_dds42.lock";
  const int lock_fd = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (lock_fd < 0) {
    fprintf(stderr, "cannot open policy bridge lock: %s\n", std::strerror(errno));
    return 1;
  }
  if (flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
    fprintf(stderr, "policy bridge already running or lock unavailable: %s\n", std::strerror(errno));
    close(lock_fd);
    return 1;
  }
  rclcpp::init(argc, argv);
  try{
    rclcpp::spin(std::make_shared<PolicyNode>());
  }catch(const std::exception&e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    close(lock_fd);
    return 1;
  } rclcpp::shutdown();
  close(lock_fd);
}
