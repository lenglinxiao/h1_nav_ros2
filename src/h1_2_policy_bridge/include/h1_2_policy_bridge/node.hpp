#pragma once
#include "h1_2_policy_bridge/core.hpp"
#include "h1_2_policy_bridge/hardware.hpp"
#include "h1_2_policy_bridge/io.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
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
#include <sstream>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
using namespace std::chrono_literals;
using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
namespace h1_policy {
inline rcl_interfaces::msg::ParameterDescriptor startup_parameter_descriptor() {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Startup configuration; use revoke_output to withdraw output authorization";
  return descriptor;
}
class ProcessLock {
public:
  ~ProcessLock() { if (fd_ >= 0) close(fd_); }
  void acquire(int domain) {
    const std::string path = "/tmp/h1_2_policy_bridge_" + std::to_string(getuid()) + "_dds" + std::to_string(domain) + ".lock";
    fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) != 0) throw std::runtime_error("policy bridge lock unavailable");
  }
private: int fd_ = -1;
};
class PolicyNode : public rclcpp::Node {
public:
PolicyNode(const rclcpp::NodeOptions& options, std::unique_ptr<LowLevelIo> io, std::function<double()> clock)
 : Node("h1_2_policy_node", options), controller_(declare_parameter<std::string>("model_path", "/home/llx/policy.onnx", startup_parameter_descriptor())), clock_(std::move(clock)), io_(std::move(io)) {
  if (!io_ || !clock_) throw std::runtime_error("IO and monotonic clock required");
  auto profile = declare_parameter<std::string>("profile", "sim", startup_parameter_descriptor());
  hardware_ = profile == "hardware";
  auto interface = declare_parameter<std::string>("network_interface", "lo", startup_parameter_descriptor());
  auto domain = declare_parameter<int>("dds_domain", 42, startup_parameter_descriptor());
  if(!h1_policy::valid_profile(profile, interface, domain)) throw std::runtime_error("invalid profile, interface or domain");
  output_enabled_ = declare_parameter<bool>("enable_hardware_output", false, startup_parameter_descriptor());
  debug_confirmed_ = declare_parameter<bool>("operator_confirmed_debug_mode", false, startup_parameter_descriptor());
  exclusive_confirmed_ = declare_parameter<bool>("operator_confirmed_exclusive_control", false, startup_parameter_descriptor());
  max_policy_steps_ = declare_parameter<int>("max_policy_steps", 0, startup_parameter_descriptor());
  if (max_policy_steps_ < 0) throw std::runtime_error("max_policy_steps cannot be negative");
  lock_.acquire(domain);
  auto topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_final", startup_parameter_descriptor());
  io_->start(domain, interface, [this](const LS& msg) {
      std::lock_guard<std::mutex> l(mutex_);

      if (hardware_) {
        if (!h1_policy::valid_crc(msg, msg.crc())) {
          has_state_ = false; fault_.mark();
          return;
        }
        h1_policy::State candidate;
        for (int i = 0; i < 27; ++i) { candidate.q[i] = msg.motor_state()[i].q(); candidate.dq[i] = msg.motor_state()[i].dq(); }
        for (int i = 0; i < 3; ++i) candidate.gyro[i] = msg.imu_state().gyroscope()[i];
        auto quat = msg.imu_state().quaternion();
        if (!h1_policy::valid_low_state(candidate, {quat[0], quat[1], quat[2], quat[3]}, msg.mode_machine(), msg.mode_pr(), 6, true)) {
          fault_.mark(); has_state_ = false; return;
        }
        auto progress = progress_.update(msg.tick(), now_seconds());
        if (progress == h1_policy::TickResult::Backward) { fault_.mark(); has_state_ = false; return; }
        if (progress == h1_policy::TickResult::Duplicate) return;
      }
      state_msg_ = msg;
      received_ = now_seconds();
      has_state_ = true;
    });
  if (!hardware_) create_publisher();
  velocity_ = create_subscription<geometry_msgs::msg::Twist>(topic, 10, [this](geometry_msgs::msg::Twist::SharedPtr m){
      cmd_ = {float(m->linear.x), float(m->linear.y), float(m->angular.z)};
      cmd_at_ = now_seconds();
    });
  for(auto item: {std::pair<const char*, h1_policy::Mode>{"passive", h1_policy::Mode::Passive}, {"fix_stand", h1_policy::Mode::FixStand}, {"arm_hold_stand", h1_policy::Mode::ArmHoldStand}, {"arm_hold", h1_policy::Mode::ArmHold}}) {
    services_.push_back(create_service<std_srvs::srv::Trigger>(std::string("~/") + item.first, [this, mode = item.second](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        if (hardware_ && !gate_.can_control()) {
          res->message = "hardware output not authorized";
          return;
        }
        h1_policy::State s;
        uint8_t mm = 0, pr = 0;
        if(!read(s, mm, pr)) {
          if (hardware_) inhibit();
          res->message = "fresh valid low state required";
          return;
        }
        if (hardware_ && mode == h1_policy::Mode::ArmHold && !budget_.can_begin()) {
          res->message = "policy budget disabled or already used; revoke and reauthorize"; return;
        }
        res->success = controller_.transition(mode, s, now_seconds());
        if(res->success) {
          if (hardware_ && mode == h1_policy::Mode::ArmHold) budget_.begin();
          active_ = true;
          res->message = "accepted";
        }else res->message = "invalid transition or readiness conditions";
      }));
  }
  services_.push_back(create_service<std_srvs::srv::Trigger>("~/status", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    h1_policy::State state; uint8_t machine = 0, pr = 0;
    bool valid = read(state, machine, pr);
    const char* mode = "Fault";
    switch (controller_.mode()) {
      case h1_policy::Mode::Passive: mode = "Passive"; break;
      case h1_policy::Mode::FixStand: mode = "FixStand"; break;
      case h1_policy::Mode::ArmHoldStand: mode = "ArmHoldStand"; break;
      case h1_policy::Mode::ArmHold: mode = "ArmHold"; break;
      case h1_policy::Mode::Fault: break;
    }
    std::ostringstream out;
    out << "profile=" << (hardware_ ? "hardware" : "sim") << " mode=" << mode
        << " authorized=" << (gate_.authorized() ? "true" : "false")
        << " shadow=" << (shadow_ ? "true" : "false")
        << " publisher_present=" << (io_->output_present() ? "true" : "false")
        << " write_count=" << write_count_ << " state=" << (valid ? "valid" : "invalid");
    res->success = true; res->message = out.str();
  }));
  if (hardware_) {
    services_.push_back(create_service<std_srvs::srv::Trigger>("~/shadow", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      h1_policy::State s; uint8_t mm = 0, pr = 0;
      if (gate_.authorized()) { res->message = "requires inhibited output"; return; }
      if (!read(s, mm, pr)) { inhibit(); res->message = "requires valid state"; return; }
      shadow_ = true; res->success = true; res->message = "shadow active; no LowCmd publisher";
    }));
    services_.push_back(create_service<std_srvs::srv::Trigger>("~/authorize_output", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      h1_policy::State s; uint8_t mm = 0, pr = 0;
      if (gate_.authorized()) { res->message = "already authorized"; return; }
      try {
        bool valid_state = read(s, mm, pr);
        if (!valid_state) { inhibit(); res->message = "fresh valid state required; output inhibited"; return; }
        if (valid_state && output_enabled_ && debug_confirmed_ && exclusive_confirmed_) {
          h1_policy::Observation obs; h1_policy::Joints action, torque;
          controller_.shadow(s, obs, action, torque);
          valid_state = read(s, mm, pr);
          if (!valid_state) { inhibit(); res->message = "state invalid after inference; output inhibited"; return; }
        }
        if (!output_enabled_ || !debug_confirmed_ || !exclusive_confirmed_) {
          res->message = "inhibited: output configuration or operator confirmations missing"; return;
        }
        // DDS writer initialization waits for discovery. Keep state callbacks
        // running during that wait; opening alone never authorizes a Write.
        create_publisher();
        bool authorized = false;
        {
          std::lock_guard<std::mutex> l(mutex_);
          valid_state = read_locked(s, mm, pr);
          authorized = gate_.authorize(output_enabled_, debug_confirmed_, exclusive_confirmed_, valid_state, [] {});
        }
        if (!authorized) {
          inhibit();
          res->message = "inhibited: configuration, operator confirmations or valid advancing state missing"; return;
        }
        shadow_ = false;
        if (!controller_.transition(h1_policy::Mode::Passive, s, now_seconds())) throw std::runtime_error("passive transition failed");
        budget_.reset(max_policy_steps_);
        active_ = true;
        res->success = true; res->message = "operator-confirmed output authorized in Passive; release not machine verified";
      } catch (const std::exception& e) { inhibit(); res->message = e.what(); }
    }));
    services_.push_back(create_service<std_srvs::srv::Trigger>("~/revoke_output", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      inhibit(); res->success = true; res->message = "publication stopped; motor behavior is not verified";
    }));
  }
  timer_ = create_wall_timer(20ms, [this] {
      tick();
    });
}
~PolicyNode() override { timer_.reset(); services_.clear(); velocity_.reset(); io_->stop(); }
private:
void create_publisher() {
  io_->open_output();
}
void inhibit() {
  active_ = false; shadow_ = false; gate_.revoke([this] { io_->close_output(); }); controller_.fault();
  std::lock_guard<std::mutex> l(mutex_);
  fault_.clear_after_inhibit();
}
double now_seconds() const { return clock_(); }
bool read(
  h1_policy::State& s, uint8_t& machine, uint8_t& pr, double* sample_received = nullptr
  ){
  std::lock_guard<std::mutex> l(mutex_);
  return read_locked(s, machine, pr, sample_received);
}
bool read_locked(h1_policy::State& s, uint8_t& machine, uint8_t& pr, double* sample_received = nullptr) {
  if (fault_.pending()) return false;
  if (!has_state_ || now_seconds() < received_ || now_seconds() - received_ > .1 || (hardware_ && !progress_.fresh(now_seconds()))) return false;
  LS msg = state_msg_;
  machine = msg.mode_machine();
  pr = msg.mode_pr();

  h1_policy::Joints q{}, dq{};
  for(int i = 0; i < 27; i++) {
    q[i] = msg.motor_state()[i].q();
    dq[i] = msg.motor_state()[i].dq();
  } s.q = h1_policy::from_sdk(q);
  s.dq = h1_policy::from_sdk(dq);
  for(int i = 0; i < 3; i++) s.gyro[i] = msg.imu_state().gyroscope()[i];

  auto quat = msg.imu_state().quaternion();
  if (!h1_policy::valid_low_state(s, {quat[0], quat[1], quat[2], quat[3]}, machine, pr, hardware_ ? 6 : 0, true)) return false;
  Eigen::Quaternionf rot(quat[0], quat[1], quat[2], quat[3]);
  auto g = rot.conjugate() * Eigen::Vector3f(0, 0, -1);
  for(int i = 0; i < 3; i++) s.gravity[i] = g[i];
  if (sample_received) *sample_received = received_;
  return true;
}
void tick(){
  if (hardware_ && shadow_) {
    h1_policy::State s; uint8_t machine = 0, pr = 0;
    if (!read(s, machine, pr)) { inhibit(); RCLCPP_ERROR(get_logger(), "shadow stopped: invalid state"); return; }
    try {
      h1_policy::Observation obs; h1_policy::Joints action, torque;
      controller_.shadow(s, obs, action, torque);
      if (++shadow_ticks_ % 50 == 0) {
        std::ostringstream line; line << "shadow zero command obs=[";
        for (auto x : obs) line << x << ',';
        line << "] action=["; for (auto x : action) line << x << ',';
        line << "] tau=["; for (auto x : torque) line << x << ',';
        line << "] ready=" << ((std::abs(s.q[3] - .9f) < .07f && std::abs(s.q[9] - .9f) < .07f && s.gravity[2] < -.9f) ? "false (manual preparation and authorization required)" : "false (knee/upright checks or manual preparation and authorization)") << " no LowCmd publisher";
        RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
      }
    } catch (const std::exception& e) { inhibit(); RCLCPP_ERROR(get_logger(), "shadow fault: %s", e.what()); }
    return;
  }
  if(!active_) return;
  h1_policy::State s;
  uint8_t machine = 0, pr = 0;
  double sample_received = 0;
  if(!read(s, machine, pr, &sample_received)) {
    if (hardware_) inhibit(); else { controller_.fault(); active_ = false; }
    RCLCPP_ERROR(get_logger(), "LowState stale or invalid; publication stopped, manual recovery required");
    return;
  }
  auto c = hardware_ ? h1_policy::Command{0, 0, 0} : cmd_;
  if(now_seconds() - cmd_at_ > .4) c = {0, 0, 0};
  for(auto x:c) if(!std::isfinite(x)) {
      if (hardware_) inhibit(); else { controller_.fault(); active_ = false; }
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
    // Hardware Passive uses motor-side damping, as specified by the supplied
    // deployment guide. It must not leave a sampled nonzero torque on exit.
    const bool motor_damping = hardware_ && controller_.mode() == h1_policy::Mode::Passive;
    const auto damping = h1_policy::to_sdk(h1_policy::passive_damping_gains());
    for(int i = 0; i < 27; i++) {
      auto& m = out.motor_cmd()[i];
      m.mode() = 1;
      m.tau() = motor_damping ? 0.f : t[i];
      m.kp() = 0;
      m.kd() = motor_damping ? damping[i] : 0.f;
    } out.crc() = crc32_core(reinterpret_cast<uint32_t*>(&out), (sizeof(out) >> 2) - 1);
    if (hardware_) {
      std::lock_guard<std::mutex> l(mutex_);
      if (!gate_.authorized() || fault_.pending() || !has_state_ ||
          now_seconds() - received_ > .1 || !progress_.fresh(now_seconds()) ||
          !h1_policy::command_snapshot_fresh(sample_received, now_seconds()))
        throw std::runtime_error("state invalid before write");
      if (!io_->write(out)) throw std::runtime_error("LowCmd write failed");
    } else if (!io_->write(out)) throw std::runtime_error("LowCmd write failed");
    ++write_count_;
    if (hardware_ && controller_.mode() == h1_policy::Mode::ArmHold && budget_.step()) {
      h1_policy::State latest; uint8_t current_machine = 0, current_pr = 0;
      if (!read(latest, current_machine, current_pr) || !controller_.transition(h1_policy::Mode::Passive, latest, now_seconds()))
        throw std::runtime_error("cannot enter Passive at policy budget exhaustion");
      RCLCPP_WARN(get_logger(), "policy budget exhausted; entering Passive damping; motor behavior not verified");
    }
  }catch(const std::exception&e) {
    if (hardware_) inhibit(); else { controller_.fault(); active_ = false; }
    RCLCPP_ERROR(get_logger(), "Policy fault: %s", e.what());
  }
}
ProcessLock lock_;
h1_policy::Controller controller_;
h1_policy::HardwareGate gate_;
h1_policy::PolicyBudget budget_;
int max_policy_steps_ = 0;
h1_policy::TickProgress progress_;
bool hardware_ = false, output_enabled_ = false, debug_confirmed_ = false, exclusive_confirmed_ = false, shadow_ = false;
unsigned shadow_ticks_ = 0;
uint64_t write_count_ = 0;
bool active_ = false, has_state_ = false;
h1_policy::FaultLatch fault_;
std::mutex mutex_;
LS state_msg_{};
double received_ = 0, cmd_at_ = 0;
h1_policy::Command cmd_{};
std::function<double()> clock_;
std::unique_ptr<LowLevelIo> io_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_;
std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace h1_policy
