#pragma once
#include "h1_2_policy_bridge/core.hpp"
#include "h1_2_policy_bridge/hardware.hpp"
#include "h1_2_policy_bridge/io.hpp"
#include "h1_2_policy_bridge/trained_guard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <h1_2_policy_interfaces/srv/claim_policy_input.hpp>
#include <rmw/types.h>
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
#include <memory>
#include <mutex>
#include <array>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
using namespace std::chrono_literals;
using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
namespace h1_policy {
inline bool valid_owner_boot_id(const std::string& value) {
  return value.size() == 32 && std::all_of(value.begin(), value.end(), [](unsigned char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}
inline bool nonzero_ros_gid(const std::vector<uint8_t>& gid) {
  return gid.size() == RMW_GID_STORAGE_SIZE &&
      std::any_of(gid.begin(), gid.end(), [](uint8_t byte) { return byte != 0; });
}
inline bool matches_ros_gid(const std::array<uint8_t, RMW_GID_STORAGE_SIZE>& expected,
                            const rmw_gid_t& actual) {
  return std::equal(expected.begin(), expected.end(), actual.data);
}
inline std::string ros_gid_hex(const std::array<uint8_t, RMW_GID_STORAGE_SIZE>& gid) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (uint8_t byte : gid) out << std::setw(2) << static_cast<unsigned>(byte);
  return out.str();
}
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
 : Node("h1_2_policy_node", options), controller_(declare_parameter<std::string>("model_path", "/home/llx/policy.onnx", startup_parameter_descriptor()), declare_parameter<double>("arm_hold_knee_tolerance", .07, startup_parameter_descriptor())), clock_(std::move(clock)), io_(std::move(io)) {
  if (!io_ || !clock_) throw std::runtime_error("IO and monotonic clock required");
  auto profile = declare_parameter<std::string>("profile", "sim", startup_parameter_descriptor());
  hardware_ = profile == "hardware";
  auto interface = declare_parameter<std::string>("network_interface", "lo", startup_parameter_descriptor());
  auto domain = declare_parameter<int>("dds_domain", 42, startup_parameter_descriptor());
  if(!h1_policy::valid_profile(profile, interface, domain)) throw std::runtime_error("invalid profile, interface or domain");
  joint_command_mode_ = declare_parameter<std::string>("joint_command_mode", "torque", startup_parameter_descriptor());
  if (joint_command_mode_ != "torque" && joint_command_mode_ != "position_pd")
    throw std::runtime_error("joint_command_mode must be torque or position_pd");
  protection_profile_ = declare_parameter<std::string>("protection_profile", "legacy", startup_parameter_descriptor());
  auto speeds = declare_parameter<std::vector<double>>("train_guard_speed_limits_sdk", {}, startup_parameter_descriptor());
  auto lower = declare_parameter<std::vector<double>>("train_guard_position_lower_sdk", {}, startup_parameter_descriptor());
  auto upper = declare_parameter<std::vector<double>>("train_guard_position_upper_sdk", {}, startup_parameter_descriptor());
  auto defaults = declare_parameter<std::vector<double>>("train_guard_default_positions_sdk", {}, startup_parameter_descriptor());
  auto names = declare_parameter<std::vector<std::string>>("train_guard_joint_names_sdk", {}, startup_parameter_descriptor());
  guard_hash_ = declare_parameter<std::string>("train_guard_profile_hash", "", startup_parameter_descriptor());
  if (protection_profile_ != "legacy" && protection_profile_ != "train_20261106")
    throw std::runtime_error("invalid protection_profile");
  if (protection_profile_ == "train_20261106") {
    if (!hardware_ || joint_command_mode_ != "position_pd")
      throw std::runtime_error("training guard requires hardware position_pd");
    if (speeds.size() != 27 || lower.size() != 27 || upper.size() != 27 || defaults.size() != 27 || names.size() != 27 || guard_hash_.empty() || guard_hash_.find_first_of(" \t\r\n") != std::string::npos)
      throw std::runtime_error("training guard configuration incomplete");
    h1_policy::TrainedGuardConfig config;
    const auto expected_defaults = h1_policy::to_sdk(h1_policy::defaults());
    for (int i = 0; i < 27; ++i) {
      if (names[i] != h1_policy::sdk_joint_names()[i] || !std::isfinite(defaults[i]) || std::abs(defaults[i] - expected_defaults[i]) > 1e-6 ||
          !std::isfinite(speeds[i]) || !std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
          speeds[i] <= 0 || lower[i] >= upper[i])
        throw std::runtime_error("invalid training guard joint configuration at SDK index " + std::to_string(i));
      config.defaults[i] = defaults[i]; config.joint_speed_limits[i] = speeds[i];
      config.pos_lower[i] = lower[i]; config.pos_upper[i] = upper[i];
    }
    trained_guard_ = std::make_unique<h1_policy::TrainedGuard>(config);
    controller_.set_training_stand_target(h1_policy::from_sdk(config.defaults));
  }
  if (hardware_ && joint_command_mode_ == "position_pd")
    RCLCPP_WARN(get_logger(), "position_pd uses motor-side PD without software torque clipping; torque limits and historical deployment configuration are unverified");
  output_enabled_ = declare_parameter<bool>("enable_hardware_output", false, startup_parameter_descriptor());
  debug_confirmed_ = declare_parameter<bool>("operator_confirmed_debug_mode", false, startup_parameter_descriptor());
  exclusive_confirmed_ = declare_parameter<bool>("operator_confirmed_exclusive_control", false, startup_parameter_descriptor());
  max_policy_steps_ = declare_parameter<int>("max_policy_steps", 0, startup_parameter_descriptor());
  if (max_policy_steps_ < 0) throw std::runtime_error("max_policy_steps cannot be negative");
  navigation_enabled_ = declare_parameter<bool>("enable_navigation_commands", false, startup_parameter_descriptor());
  continuous_navigation_ = declare_parameter<bool>("continuous_policy_navigation", false, startup_parameter_descriptor());
  if (continuous_navigation_ && (!hardware_ || !navigation_enabled_ || joint_command_mode_ != "position_pd" || max_policy_steps_ != 0))
    throw std::runtime_error("continuous navigation requires hardware, navigation enabled, position_pd and zero diagnostic budget");
  max_policy_seconds_ = declare_parameter<double>("max_policy_seconds", 0.0, startup_parameter_descriptor());
  if (!std::isfinite(max_policy_seconds_) || max_policy_seconds_ < 0 || max_policy_seconds_ > 3.1)
    throw std::runtime_error("max_policy_seconds must be finite and between 0 and 3.1");
  if (max_policy_seconds_ > 0 && (!hardware_ || joint_command_mode_ != "position_pd" ||
                                   continuous_navigation_ || max_policy_steps_ == 0))
    throw std::runtime_error("max_policy_seconds requires hardware position_pd with a finite policy step budget");
  input_identity_mode_ = declare_parameter<std::string>("input_identity_mode", "legacy", startup_parameter_descriptor());
  owner_boot_id_ = declare_parameter<std::string>("owner_boot_id", "", startup_parameter_descriptor());
  if (input_identity_mode_ != "legacy" && input_identity_mode_ != "claimed")
    throw std::runtime_error("input_identity_mode must be legacy or claimed");
  if (input_identity_mode_ == "claimed" && !h1_policy::valid_owner_boot_id(owner_boot_id_))
    throw std::runtime_error("claimed input_identity_mode requires a 32-character lowercase hexadecimal owner_boot_id");
  claimed_mode_ = input_identity_mode_ == "claimed";
  ready_publisher_ = rclcpp::Node::create_publisher<std_msgs::msg::Bool>("/h1_2/policy_ready", 10);
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
  velocity_ = create_subscription<geometry_msgs::msg::Twist>(topic, 10, [this](geometry_msgs::msg::Twist::SharedPtr m, const rclcpp::MessageInfo& info){
      if (claimed_mode_ && (!input_claimed_ || !h1_policy::matches_ros_gid(cmd_writer_gid_, info.get_rmw_message_info().publisher_gid))) return;
      cmd_ = {float(m->linear.x), float(m->linear.y), float(m->angular.z)};
      cmd_at_ = now_seconds();
      if (claimed_mode_) cmd_after_claim_ = true;
    });
  gate_heartbeat_ = create_subscription<std_msgs::msg::Bool>("/h1_2/policy_gate_alive", 10, [this](std_msgs::msg::Bool::SharedPtr msg, const rclcpp::MessageInfo& info) {
      if (claimed_mode_ && (!input_claimed_ || !h1_policy::matches_ros_gid(heartbeat_writer_gid_, info.get_rmw_message_info().publisher_gid))) return;
      gate_alive_ = msg->data;
      gate_at_ = now_seconds();
      if (claimed_mode_) heartbeat_after_claim_ = true;
      if (!gate_alive_) publish_not_ready();
    });
  if (claimed_mode_) {
    claim_input_service_ = create_service<h1_2_policy_interfaces::srv::ClaimPolicyInput>("~/claim_input",
        [this](const std::shared_ptr<h1_2_policy_interfaces::srv::ClaimPolicyInput::Request> request,
               std::shared_ptr<h1_2_policy_interfaces::srv::ClaimPolicyInput::Response> response) {
          const bool valid = request->owner_boot_id == owner_boot_id_ &&
              h1_policy::nonzero_ros_gid(request->cmd_writer_gid) &&
              h1_policy::nonzero_ros_gid(request->heartbeat_writer_gid);
          if (!valid) {
            input_claimed_ = false;
            cmd_writer_gid_.fill(0);
            heartbeat_writer_gid_.fill(0);
            cmd_ = {0, 0, 0}; cmd_at_ = 0;
            last_policy_command_ = {0, 0, 0};
            gate_alive_ = false; gate_at_ = 0;
            cmd_after_claim_ = false; heartbeat_after_claim_ = false;
            publish_not_ready();
            response->success = false;
            response->reason = "CLAIM_REJECTED; previous input lease cleared";
            return;
          }
          std::copy(request->cmd_writer_gid.begin(), request->cmd_writer_gid.end(), cmd_writer_gid_.begin());
          std::copy(request->heartbeat_writer_gid.begin(), request->heartbeat_writer_gid.end(), heartbeat_writer_gid_.begin());
          cmd_ = {0, 0, 0}; cmd_at_ = 0;
          last_policy_command_ = {0, 0, 0};
          gate_alive_ = false; gate_at_ = 0;
          cmd_after_claim_ = false; heartbeat_after_claim_ = false;
          input_claimed_ = true;
          publish_not_ready();
          response->success = true;
          response->reason = "CLAIMED; matching command and heartbeat required";
        });
  }
  for(auto item: {std::pair<const char*, h1_policy::Mode>{"passive", h1_policy::Mode::Passive}, {"fix_stand", h1_policy::Mode::FixStand}, {"arm_hold_stand", h1_policy::Mode::ArmHoldStand}, {"arm_hold", h1_policy::Mode::ArmHold}}) {
    services_.push_back(create_service<std_srvs::srv::Trigger>(std::string("~/") + item.first, [this, mode = item.second](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        if (hardware_ && !gate_.can_control()) {
          res->message = "hardware output not authorized";
          return;
        }
        if (trained_guard_ && trained_guard_->tripped() && mode != h1_policy::Mode::Passive) {
          res->message = "training guard latched; revoke and reauthorize before active modes"; return;
        }
        h1_policy::State s;
        uint8_t mm = 0, pr = 0;
        if(!read(s, mm, pr)) {
          if (hardware_) inhibit();
          res->message = "fresh valid low state required";
          return;
        }
        if (transition_ == Transition::Hold || transition_ == Transition::Exit) {
          res->success = mode == h1_policy::Mode::Passive;
          res->message = res->success ? "hold or supported release already in progress" : "hold or supported release in progress";
          return;
        }
        if (controlled_pd() && !have_last_control_ && mode != h1_policy::Mode::Passive) {
          res->message = "waiting for first successful Passive write"; return;
        }
        if (transition_ == Transition::Entry && mode != h1_policy::Mode::Passive) {
          res->message = "entry transition in progress"; return;
        }
        if (controlled_pd() && mode == h1_policy::Mode::Passive && controller_.mode() != h1_policy::Mode::Passive && have_last_control_) {
          try {
            begin_hold(s);
            res->success = true; res->message = "holding output; external support required before release";
          } catch (const std::exception& e) { inhibit(); res->message = e.what(); }
          return;
        }
        if (hardware_ && mode == h1_policy::Mode::ArmHold && !continuous_navigation_ && !budget_.can_begin()) {
          res->message = "policy budget disabled or already used; revoke and reauthorize"; return;
        }
        if (trained_guard_ && mode == h1_policy::Mode::ArmHold) {
          auto sdk_state = s; sdk_state.q = h1_policy::to_sdk(s.q); sdk_state.dq = h1_policy::to_sdk(s.dq);
          auto prearm = trained_guard_->prearm(sdk_state);
          guard_prearm_max_ = prearm.max_pos_error; guard_prearm_index_ = prearm.sdk_index;
        }
        res->success = controller_.transition(mode, s, now_seconds(), !trained_guard_);
        if(res->success) {
          if (trained_guard_ && mode == h1_policy::Mode::ArmHold) { trained_guard_->arm(now_seconds()); guard_armed_ = true; }
          if (hardware_ && mode == h1_policy::Mode::ArmHold && !continuous_navigation_) {
            budget_.begin();
            if (max_policy_seconds_ > 0) policy_deadline_ = now_seconds() + max_policy_seconds_;
          }
          if (controlled_pd() && mode != h1_policy::Mode::Passive && have_last_control_) {
            transition_ = Transition::Entry; transition_from_ = last_control_; transition_started_ = false;
            for (int i = 0; i < 27; ++i) if (transition_from_.kp[i] == 0.f) transition_from_.q[i] = s.q[i];
          }
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
    if (transition_ == Transition::Exit) mode = "Stopping";
    if (transition_ == Transition::Hold) mode = "Hold";
    std::ostringstream out;
    out << "profile=" << (hardware_ ? "hardware" : "sim") << " mode=" << mode
        << " protection_profile=" << protection_profile_ << " train_guard_profile_hash=" << (guard_hash_.empty() ? "none" : guard_hash_)
        << " trained_guard_tripped=" << (trained_guard_ && trained_guard_->tripped() ? "true" : "false")
        << " trained_guard_native_damping=" << (trained_guard_ && trained_guard_->tripped() && active_ && gate_.authorized() && controller_.mode() == h1_policy::Mode::Passive && transition_ == Transition::None ? "true" : "false")
        << " guard_prearm_max_delta=" << guard_prearm_max_ << " guard_prearm_sdk_index=" << guard_prearm_index_
        << " " << guard_event_status_
        << " joint_command_mode=" << joint_command_mode_
        << " max_policy_seconds=" << max_policy_seconds_
        << " software_torque_clamp=" << (joint_command_mode_ == "torque" ? "true" : "false")
        << " position_pd_sample_torque_projection=" << (controlled_pd() ? "true" : "false")
        << " controlled_stop_supported=false"
        << " exit_protocol=" << (controlled_pd() ? "hold_supported_release_v3" : "legacy")
        << " supported_release_required=" << (controlled_pd() ? "true" : "false")
        << " release_blocked=" << (release_blocked_ ? "true" : "false")
        << " release_kp_zero=" << (release_blocked_ && std::all_of(last_control_.kp.begin(), last_control_.kp.end(), [](float kp) { return kp == 0.f; }) ? "true" : "false")
        << " release_phase=" << release_phase_name()
        << " " << last_release_block_
        << " limited_frames=" << limited_frames_
        << " output_transition=" << (transition_ == Transition::Entry ? "entry" : transition_ == Transition::Exit ? "exit" : transition_ == Transition::Hold ? "hold" : "none")
        << " authorized=" << (gate_.authorized() ? "true" : "false")
        << " navigation_enabled=" << (navigation_enabled_ ? "true" : "false")
        << " input_identity_mode=" << input_identity_mode_
        << " input_claimed=" << (input_claimed_ ? "true" : "false")
        << " owner_boot_id=" << (owner_boot_id_.empty() ? "none" : owner_boot_id_)
        << " cmd_writer_gid=" << (input_claimed_ ? h1_policy::ros_gid_hex(cmd_writer_gid_) : "none")
        << " heartbeat_writer_gid=" << (input_claimed_ ? h1_policy::ros_gid_hex(heartbeat_writer_gid_) : "none")
        << " continuous_navigation=" << (continuous_navigation_ ? "true" : "false")
        << " entry_angle_gate=" << (trained_guard_ ? "diagnostic_only" : "enforced")
        << " arm_hold_knee_tolerance=" << get_parameter("arm_hold_knee_tolerance").as_double()
        << " navigation_ready=" << (ready_ ? "true" : "false")
        << " policy_cmd=" << last_policy_command_[0] << "," << last_policy_command_[1] << "," << last_policy_command_[2]
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
        last_release_block_ = "last_release_block_reason=none";
        if (trained_guard_) { trained_guard_->reset(); guard_armed_ = false; guard_event_status_ = "trained_guard_reason=none trained_guard_sdk_index=-1 trained_guard_value=0 trained_guard_limit=0 trained_guard_tick=0 trained_guard_elapsed=0 trained_guard_warmup=false"; guard_prearm_max_ = 0; guard_prearm_index_ = -1; }
        shadow_ = false;
        if (!controller_.transition(h1_policy::Mode::Passive, s, now_seconds())) throw std::runtime_error("passive transition failed");
        budget_.reset(max_policy_steps_);
        active_ = true;
        res->success = true; res->message = "operator-confirmed output authorized in Passive; release not machine verified";
      } catch (const std::exception& e) { inhibit(); res->message = e.what(); }
    }));
    services_.push_back(create_service<std_srvs::srv::Trigger>("~/release_supported", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      if (!controlled_pd() || !gate_.authorized() || transition_ != Transition::Hold) {
        res->message = "requires authorized position PD Hold"; return;
      }
      if (release_blocked_) {
        res->message = "release blocked; explicit physical handoff and revocation required"; return;
      }
      h1_policy::State state; uint8_t mm = 0, pr = 0;
      if (!read(state, mm, pr)) { inhibit(); res->message = "fresh valid state required; output stopped"; return; }
      try {
        begin_exit(state); res->success = true;
        res->message = "caller asserted external support; supported release started; support is not machine verified";
      } catch (const std::exception& e) { inhibit(); res->message = e.what(); }
    }));
    services_.push_back(create_service<std_srvs::srv::Trigger>("~/revoke_output", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      inhibit(); res->success = true; res->message = "publication stopped; motor behavior is not verified";
    }));
  }
  timer_ = create_wall_timer(20ms, [this] {
      tick();
      publish_readiness();
    });
}
~PolicyNode() override { timer_.reset(); services_.clear(); velocity_.reset(); io_->stop(); }
private:
enum class Transition { None, Entry, Hold, Exit };
bool controlled_pd() const { return hardware_ && joint_command_mode_ == "position_pd"; }
const char* release_phase_name() const {
  if (release_blocked_) return "blocked";
  if (transition_ != Transition::Exit) return "none";
  switch (release_phase_) {
    case h1_policy::GuardedReleasePhase::Unloading: return "unloading";
    case h1_policy::GuardedReleasePhase::Settling: return "settling";
    case h1_policy::GuardedReleasePhase::DampingDown: return "damping_down";
    case h1_policy::GuardedReleasePhase::Blocked: return "blocked";
    case h1_policy::GuardedReleasePhase::Done: return "done";
  }
  return "invalid";
}
void record_release_block(const h1_policy::ReleaseBlockInfo& block, const char* source) {
  const char* reason = block.reason == h1_policy::ReleaseBlockReason::Both ? "both" :
      block.reason == h1_policy::ReleaseBlockReason::Speed ? "speed" : "timeout";
  const char* phase = block.previous_phase == h1_policy::GuardedReleasePhase::Unloading ? "unloading" :
      block.previous_phase == h1_policy::GuardedReleasePhase::Settling ? "settling" : "damping_down";
  const int i = block.policy_index;
  const int sdk_index = i == 17 ? 18 : i == 18 ? 17 : i == 24 ? 25 : i == 25 ? 24 : i;
  std::ostringstream out;
  out.precision(9);
  out << "last_release_block_reason=" << reason << " last_release_block_source=" << source
      << " last_release_block_phase=" << phase << " last_release_block_elapsed=" << block.elapsed
      << " last_release_block_policy_index=" << i << " last_release_block_sdk_index=" << sdk_index << " last_release_block_q=" << block.q
      << " last_release_block_dq=" << block.dq << " last_release_block_max_abs_dq=" << block.max_abs_dq
      << " last_release_block_cmd_q=" << last_control_.q[i] << " last_release_block_cmd_kp=" << last_control_.kp[i]
      << " last_release_block_cmd_kd=" << last_control_.kd[i];
  last_release_block_ = out.str();
  RCLCPP_WARN(get_logger(), "%s", last_release_block_.c_str());
}
bool policy_time_expired() const {
  return controlled_pd() && !continuous_navigation_ && max_policy_seconds_ > 0 &&
      controller_.mode() == h1_policy::Mode::ArmHold && now_seconds() >= policy_deadline_;
}
void begin_hold(const h1_policy::State& s) {
  transition_from_ = last_control_;
  transition_ = Transition::Hold; transition_started_ = false;
  if (!controller_.transition(h1_policy::Mode::Passive, s, now_seconds())) throw std::runtime_error("cannot begin hold");
  RCLCPP_WARN(get_logger(), "holding last output; external support required before release; physical safety not verified");
}
void begin_exit(const h1_policy::State& s) {
  release_ = std::make_unique<h1_policy::GuardedRelease>(last_control_, s, now_seconds());
  release_phase_ = h1_policy::GuardedReleasePhase::Unloading;
  transition_ = Transition::Exit; transition_started_ = false;
  if (!controller_.transition(h1_policy::Mode::Passive, s, now_seconds())) throw std::runtime_error("cannot begin exit");
  RCLCPP_WARN(get_logger(), "controlled exit requested; physical safety and motor torque not verified");
}
void create_publisher() {
  io_->open_output();
}
void inhibit() {
  publish_not_ready();
  last_policy_command_ = {0, 0, 0};
  active_ = false; shadow_ = false; transition_ = Transition::None; have_last_control_ = false;
  release_.reset(); release_blocked_ = false;
  gate_.revoke([this] { io_->close_output(); }); controller_.fault();
  std::lock_guard<std::mutex> l(mutex_);
  fault_.clear_after_inhibit();
}
void publish_not_ready() {
  ready_ = false;
  if (ready_publisher_) { std_msgs::msg::Bool msg; msg.data = false; ready_publisher_->publish(msg); }
}
bool gate_fresh() const { return gate_alive_ && now_seconds() >= gate_at_ && now_seconds() - gate_at_ < .2; }
void publish_readiness() {
  h1_policy::State s; uint8_t machine = 0, pr = 0;
  const bool next = navigation_enabled_ && active_ && controller_.mode() == h1_policy::Mode::ArmHold &&
      transition_ == Transition::None && (!claimed_mode_ || (input_claimed_ && cmd_after_claim_ && heartbeat_after_claim_)) &&
      (hardware_ ? (continuous_navigation_ && gate_.authorized() && gate_fresh()) : (!claimed_mode_ || gate_fresh())) &&
      io_->output_present() && write_count_ > 0 && read(s, machine, pr);
  if (next && !ready_) ready_since_ = now_seconds();
  ready_ = next;
  std_msgs::msg::Bool msg; msg.data = next; ready_publisher_->publish(msg);
}
double now_seconds() const { return clock_(); }
bool read(
  h1_policy::State& s, uint8_t& machine, uint8_t& pr, double* sample_received = nullptr, uint32_t* sample_tick = nullptr
  ){
  std::lock_guard<std::mutex> l(mutex_);
  return read_locked(s, machine, pr, sample_received, sample_tick);
}
bool read_locked(h1_policy::State& s, uint8_t& machine, uint8_t& pr, double* sample_received = nullptr, uint32_t* sample_tick = nullptr) {
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
  if (sample_tick) *sample_tick = msg.tick();
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
        line << "] ready=" << ((std::abs(s.q[3] - .9f) < get_parameter("arm_hold_knee_tolerance").as_double() && std::abs(s.q[9] - .9f) < get_parameter("arm_hold_knee_tolerance").as_double() && s.gravity[2] < -.9f) ? "false (manual preparation and authorization required)" : "false (knee/upright checks or manual preparation and authorization)") << " no LowCmd publisher";
        RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
      }
    } catch (const std::exception& e) { inhibit(); RCLCPP_ERROR(get_logger(), "shadow fault: %s", e.what()); }
    return;
  }
  if(!active_) return;
  h1_policy::State s;
  uint8_t machine = 0, pr = 0;
  double sample_received = 0;
  uint32_t sample_tick = 0;
  if(!read(s, machine, pr, &sample_received, &sample_tick)) {
    if (hardware_) inhibit(); else { controller_.fault(); active_ = false; }
    RCLCPP_ERROR(get_logger(), "LowState stale or invalid; publication stopped, manual recovery required");
    return;
  }
  auto c = hardware_ ? h1_policy::Command{0, 0, 0} : cmd_;
  if (claimed_mode_ && !hardware_ &&
      (!input_claimed_ || !ready_ || !gate_fresh() || cmd_at_ <= ready_since_)) c = {0, 0, 0};
  if (hardware_ && navigation_enabled_ && continuous_navigation_ && ready_ && gate_fresh() && cmd_at_ > ready_since_)
    c = cmd_;
  if(now_seconds() < cmd_at_ || now_seconds() - cmd_at_ > .4) c = {0, 0, 0};
  for(auto x:c) if(!std::isfinite(x)) {
      if (hardware_) inhibit(); else { controller_.fault(); active_ = false; }
      return;
    }
  c[0] = std::clamp(c[0], -.2f, .2f);
  c[1] = 0;
  c[2] = std::clamp(c[2], -.5f, .5f);
  last_policy_command_ = controller_.mode() == h1_policy::Mode::ArmHold ? c : h1_policy::Command{0, 0, 0};
  try{
    if (trained_guard_ && guard_armed_ && !trained_guard_->tripped()) {
      auto sdk_state = s; sdk_state.q = h1_policy::to_sdk(s.q); sdk_state.dq = h1_policy::to_sdk(s.dq);
      auto event = trained_guard_->observe(sdk_state, sample_tick, now_seconds());
      if (event) {
        const char* reason = event->reason == h1_policy::TrainedGuardReason::JointSpeed ? "joint_speed" :
            event->reason == h1_policy::TrainedGuardReason::JointPosition ? "joint_position" :
            event->reason == h1_policy::TrainedGuardReason::BaseAngularSpeed ? "base_angular_speed" : "tilt";
        std::ostringstream out; out.precision(9);
        out << "trained_guard_reason=" << reason << " trained_guard_sdk_index=" << event->sdk_index
            << " trained_guard_value=" << event->value << " trained_guard_limit=" << event->limit
            << " trained_guard_tick=" << event->tick << " trained_guard_elapsed=" << event->elapsed
            << " trained_guard_warmup=" << (event->warmup ? "true" : "false");
        guard_event_status_ = out.str();
        publish_not_ready(); cmd_ = {0, 0, 0}; last_policy_command_ = {0, 0, 0};
        transition_ = Transition::None; transition_started_ = false; release_.reset(); release_blocked_ = false;
        if (!controller_.transition(h1_policy::Mode::Passive, s, now_seconds())) throw std::runtime_error("guard Passive transition failed");
        RCLCPP_WARN(get_logger(), "training guard latched; native damping requested: %s", guard_event_status_.c_str());
      }
    }
    if (policy_time_expired()) {
      begin_hold(s);
      RCLCPP_WARN(get_logger(), "policy time budget exhausted; holding last output; external support required");
    }
    const bool policy_step = controller_.mode() == h1_policy::Mode::ArmHold && transition_ != Transition::Exit && transition_ != Transition::Hold;
    h1_policy::JointControl control{};
    std::unique_ptr<h1_policy::GuardedRelease> next_release;
    bool transition_finished = false;
    if (transition_ == Transition::Exit) {
      if (!release_) throw std::runtime_error("missing guarded release state");
      // Commit release progress only after a successful write. A newer state
      // can invalidate this tentative step before it reaches the motors.
      next_release = std::make_unique<h1_policy::GuardedRelease>(*release_);
      const auto result = next_release->step(s, now_seconds());
      release_phase_ = result.phase;
      if (result.blocked) {
        record_release_block(result.block, "snapshot");
        begin_hold(s);
        release_blocked_ = true;
        release_.reset();
        next_release.reset();
        control = transition_from_;
        RCLCPP_WARN(get_logger(), "release blocked; holding last output; position support is not guaranteed; explicit physical handoff and revocation required");
      } else {
        control = result.control;
        transition_finished = result.done;
      }
    } else if (transition_ == Transition::Hold) {
      control = transition_from_;
    } else {
      control = controller_.step_control(s, c, now_seconds());
    }
    if (transition_ == Transition::Entry) {
      if (!transition_started_) { transition_start_ = now_seconds(); transition_started_ = true; }
      const float progress = std::clamp(float((now_seconds() - transition_start_) / 3.0), 0.f, 1.f);
      control = h1_policy::blend_position_pd(transition_from_, control, progress);
      transition_finished = progress >= 1.f;
    }
    bool limited = false;
    int limited_joint = -1; float raw_q = 0.f;
    if (controlled_pd() && (controller_.mode() != h1_policy::Mode::Passive || transition_ == Transition::Exit || transition_ == Transition::Hold)) {
      auto projected = h1_policy::project_position_pd(s, control);
      for (int i = 0; i < 27; ++i) if (projected.limited[i]) { limited_joint = i; raw_q = control.q[i]; break; }
      control = projected.control;
      limited = limited_joint >= 0;
    }
    auto t = h1_policy::to_sdk(control.tau);
    auto q = h1_policy::to_sdk(control.q);
    auto kp = h1_policy::to_sdk(control.kp);
    auto kd = h1_policy::to_sdk(control.kd);
    for (float torque : t) {
      if (!std::isfinite(torque)) throw std::runtime_error("non-finite torque");
    }
    if (joint_command_mode_ == "position_pd")
      for (int i = 0; i < 27; ++i)
        if (!std::isfinite(q[i]) || !std::isfinite(kp[i]) || !std::isfinite(kd[i]))
          throw std::runtime_error("non-finite position PD command");
    LC out{};
    out.mode_machine() = machine;
    out.mode_pr() = pr;
    // Hardware Passive uses motor-side damping, as specified by the supplied
    // deployment guide. It must not leave a sampled nonzero torque on exit.
    const bool motor_damping = hardware_ && controller_.mode() == h1_policy::Mode::Passive && transition_ != Transition::Exit && transition_ != Transition::Hold;
    const auto damping = h1_policy::to_sdk(h1_policy::passive_damping_gains());
    for(int i = 0; i < 27; i++) {
      auto& m = out.motor_cmd()[i];
      m.mode() = 1;
      m.tau() = (motor_damping || joint_command_mode_ == "position_pd") ? 0.f : t[i];
      m.q() = !motor_damping && joint_command_mode_ == "position_pd" ? q[i] : 0.f;
      m.kp() = !motor_damping && joint_command_mode_ == "position_pd" ? kp[i] : 0.f;
      m.kd() = motor_damping ? damping[i] : (joint_command_mode_ == "position_pd" ? kd[i] : 0.f);
    } out.crc() = crc32_core(reinterpret_cast<uint32_t*>(&out), (sizeof(out) >> 2) - 1);
    if (hardware_) {
      std::lock_guard<std::mutex> l(mutex_);
      if (!gate_.authorized() || fault_.pending() || !has_state_ ||
          now_seconds() - received_ > .1 || !progress_.fresh(now_seconds()) ||
          !h1_policy::command_snapshot_fresh(sample_received, now_seconds()))
        throw std::runtime_error("state invalid before write");
      // A newer sample can arrive while calculating the outgoing command.
      // Never send a further unloading step when that sample is already fast.
      if (transition_ == Transition::Exit) {
        h1_policy::State latest; uint8_t latest_machine = 0, latest_pr = 0;
        if (!read_locked(latest, latest_machine, latest_pr))
          throw std::runtime_error("release state invalid before write");
        // Recheck both speed and total release time against the latest sample.
        // Neither candidate is committed until a write succeeds.
        auto rechecked = std::make_unique<h1_policy::GuardedRelease>(*release_);
        const auto corrected = rechecked->step(latest, now_seconds());
        if (corrected.blocked) {
          record_release_block(corrected.block, "newer_state");
          begin_hold(latest);
          release_blocked_ = true;
          release_.reset();
          RCLCPP_WARN(get_logger(), "release blocked by newer state; holding last output; position support is not guaranteed; explicit physical handoff and revocation required");
          return;
        }
        // Settling requires continuous quiet. If a newer sample interrupts it,
        // discard the tentative output and update the committed state with the
        // newer sample instead; this also prevents a late Done transition.
        const bool quiet_phase = release_phase_ == h1_policy::GuardedReleasePhase::Settling ||
            release_phase_ == h1_policy::GuardedReleasePhase::DampingDown ||
            release_phase_ == h1_policy::GuardedReleasePhase::Done;
        const bool sampled_quiet = std::all_of(s.dq.begin(), s.dq.end(),
                [](float dq) { return std::abs(dq) < h1_policy::kReleaseQuietSpeed; });
        if (quiet_phase && sampled_quiet && std::any_of(latest.dq.begin(), latest.dq.end(),
                [](float dq) { return std::abs(dq) >= h1_policy::kReleaseQuietSpeed; })) {
          release_phase_ = corrected.phase;
          release_ = std::move(rechecked);
          return;
        }
      }
      // Inference or command preparation may cross the deadline. Preserve the
      // last successfully written control; do not send the late policy result.
      if (policy_step && policy_time_expired()) {
        begin_hold(s);
        RCLCPP_WARN(get_logger(), "policy time budget exhausted; holding last output; external support required");
        return;
      }
      if (!io_->write(out)) throw std::runtime_error("LowCmd write failed");
      if (next_release) release_ = std::move(next_release);
    } else if (!io_->write(out)) throw std::runtime_error("LowCmd write failed");
    ++write_count_;
    if (limited && (++limited_frames_ == 1 || limited_frames_ % 50 == 0))
      RCLCPP_WARN(get_logger(), "position PD sample torque projection limited_frames=%lu policy_joint=%d raw_q=%g projected_q=%g sample_tau=%g; motor torque is not guaranteed", static_cast<unsigned long>(limited_frames_), limited_joint, raw_q, control.q[limited_joint], control.tau[limited_joint]);
    if (controlled_pd()) {
      if (motor_damping) { control = {}; control.kd = h1_policy::passive_damping_gains(); }
      last_control_ = control; have_last_control_ = true;
      if (transition_finished) { transition_ = Transition::None; release_.reset(); }
    }
    if (hardware_ && policy_step && !continuous_navigation_ && budget_.step()) {
      h1_policy::State latest; uint8_t current_machine = 0, current_pr = 0;
      if (!read(latest, current_machine, current_pr)) throw std::runtime_error("invalid state at policy budget exhaustion");
      if (controlled_pd()) begin_hold(latest);
      else if (!controller_.transition(h1_policy::Mode::Passive, latest, now_seconds()))
        throw std::runtime_error("cannot enter Passive at policy budget exhaustion");
      RCLCPP_WARN(get_logger(), "%s", controlled_pd() ? "policy budget exhausted; holding last output; external support required" : "policy budget exhausted; exiting control; motor behavior not verified");
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
std::string joint_command_mode_;
std::string protection_profile_, guard_hash_;
std::unique_ptr<h1_policy::TrainedGuard> trained_guard_;
std::string guard_event_status_ = "trained_guard_reason=none trained_guard_sdk_index=-1 trained_guard_value=0 trained_guard_limit=0 trained_guard_tick=0 trained_guard_elapsed=0 trained_guard_warmup=false";
double guard_prearm_max_ = 0;
int guard_prearm_index_ = -1;
bool guard_armed_ = false;
h1_policy::TickProgress progress_;
bool hardware_ = false, output_enabled_ = false, debug_confirmed_ = false, exclusive_confirmed_ = false, shadow_ = false;
double max_policy_seconds_ = 0, policy_deadline_ = 0;
bool navigation_enabled_ = false, continuous_navigation_ = false, ready_ = false, gate_alive_ = false;
bool claimed_mode_ = false, input_claimed_ = false;
bool cmd_after_claim_ = false, heartbeat_after_claim_ = false;
std::string input_identity_mode_ = "legacy", owner_boot_id_;
std::array<uint8_t, RMW_GID_STORAGE_SIZE> cmd_writer_gid_{}, heartbeat_writer_gid_{};
double ready_since_ = 0, gate_at_ = 0;
unsigned shadow_ticks_ = 0;
Transition transition_ = Transition::None;
h1_policy::JointControl last_control_{}, transition_from_{};
bool have_last_control_ = false, transition_started_ = false;
std::unique_ptr<h1_policy::GuardedRelease> release_;
h1_policy::GuardedReleasePhase release_phase_ = h1_policy::GuardedReleasePhase::Unloading;
bool release_blocked_ = false;
std::string last_release_block_ = "last_release_block_reason=none";
double transition_start_ = 0;
uint64_t limited_frames_ = 0;
uint64_t write_count_ = 0;
bool active_ = false, has_state_ = false;
h1_policy::FaultLatch fault_;
std::mutex mutex_;
LS state_msg_{};
double received_ = 0, cmd_at_ = 0;
h1_policy::Command cmd_{}, last_policy_command_{};
std::function<double()> clock_;
std::unique_ptr<LowLevelIo> io_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_heartbeat_;
rclcpp::Service<h1_2_policy_interfaces::srv::ClaimPolicyInput>::SharedPtr claim_input_service_;
std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace h1_policy
