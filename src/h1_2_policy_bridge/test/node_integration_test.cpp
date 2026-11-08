#include "h1_2_policy_bridge/node.hpp"
#include "check.hpp"
#include <chrono>
#include <array>
#include <iomanip>
#include <thread>
#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <rmw/types.h>
#include <h1_2_policy_interfaces/srv/claim_policy_input.hpp>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using ClaimInput = h1_2_policy_interfaces::srv::ClaimPolicyInput;
using State = unitree_hg::msg::dds_::LowState_;
using Command = unitree_hg::msg::dds_::LowCmd_;

template<class Message>
std::vector<uint8_t> publisher_gid(const typename rclcpp::Publisher<Message>::SharedPtr& publisher) {
  const auto gid = publisher->get_gid();
  return std::vector<uint8_t>(gid.data, gid.data + RMW_GID_STORAGE_SIZE);
}

std::string gid_hex(const std::vector<uint8_t>& gid) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (uint8_t byte : gid) out << std::setw(2) << static_cast<unsigned>(byte);
  return out.str();
}

struct FakeIo final : h1_policy::LowLevelIo {
  StateCallback callback;
  std::function<void()> on_open;
  bool present = false, fail_write = false;
  int opens = 0, closes = 0;
  std::vector<Command> writes;
  void start(int, const std::string&, StateCallback cb) override { callback = std::move(cb); }
  void open_output() override { present = true; ++opens; if (on_open) on_open(); }
  bool write(const Command& c) override { CHECK(present); if (fail_write) return false; writes.push_back(c); return true; }
  void close_output() override { present = false; ++closes; }
  bool output_present() const override { return present; }
  void stop() override { callback = {}; }
};

struct Fixture {
  double clock = 10;
  int clock_reads = 0, jump_on_clock_read = 0;
  int inject_on_clock_read = 0;
  std::function<void()> clock_injection;
  double clock_jump = 0;
  uint32_t tick = 0;
  bool simulation_profile;
  const std::string test_namespace = "/h1_policy_test_" + std::to_string(getpid());
  FakeIo* io;
  std::shared_ptr<h1_policy::PolicyNode> node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  Fixture(const std::string& model, bool enabled = false, bool sim = false, const std::string& command_mode = "torque", int budget = -1, bool nav = false, bool continuous = false, double max_policy_seconds = 0, bool trained = false, bool claimed = false, std::string owner = "") {
    simulation_profile = sim;
    auto fake = std::make_unique<FakeIo>(); io = fake.get();
    rclcpp::NodeOptions options;
    // Service calls must never resolve to a live robot's policy node, even
    // when this binary is invoked outside the CTest environment.
    options.arguments({"--ros-args", "-r", "__ns:=" + test_namespace});
    std::vector<std::string> names; for (auto name : h1_policy::sdk_joint_names()) names.emplace_back(name);
    auto sdk_defaults = h1_policy::to_sdk(h1_policy::defaults());
    std::vector<double> defaults(sdk_defaults.begin(), sdk_defaults.end());
    options.parameter_overrides({rclcpp::Parameter("model_path", model),
      rclcpp::Parameter("profile", sim ? "sim" : "hardware"),
      rclcpp::Parameter("joint_command_mode", command_mode),
      rclcpp::Parameter("network_interface", sim ? "lo" : "eth0"),
      rclcpp::Parameter("dds_domain", sim ? 42 : 0),
      rclcpp::Parameter("enable_hardware_output", enabled),
      rclcpp::Parameter("operator_confirmed_debug_mode", enabled),
      rclcpp::Parameter("operator_confirmed_exclusive_control", enabled),
      rclcpp::Parameter("max_policy_steps", budget < 0 ? (enabled ? 2 : 0) : budget),
      rclcpp::Parameter("enable_navigation_commands", nav),
      rclcpp::Parameter("continuous_policy_navigation", continuous),
      rclcpp::Parameter("input_identity_mode", claimed ? "claimed" : "legacy"),
      rclcpp::Parameter("owner_boot_id", owner),
      rclcpp::Parameter("max_policy_seconds", max_policy_seconds),
      rclcpp::Parameter("protection_profile", trained ? "train_20261106" : "legacy"),
      rclcpp::Parameter("train_guard_speed_limits_sdk", std::vector<double>(27, 1.0)),
      rclcpp::Parameter("train_guard_position_lower_sdk", std::vector<double>(27, -3.0)),
      rclcpp::Parameter("train_guard_position_upper_sdk", std::vector<double>(27, 3.0)),
      rclcpp::Parameter("train_guard_default_positions_sdk", defaults),
      rclcpp::Parameter("train_guard_joint_names_sdk", names),
      rclcpp::Parameter("train_guard_profile_hash", "test")});
    node = std::make_shared<h1_policy::PolicyNode>(options, std::move(fake), [this] {
      ++clock_reads;
      if (clock_reads == jump_on_clock_read) clock += clock_jump;
      if (clock_reads == inject_on_clock_read && clock_injection) {
        auto injection = std::move(clock_injection);
        inject_on_clock_read = 0;
        injection();
      }
      return clock;
    });
    client_node = std::make_shared<rclcpp::Node>("integration_client", test_namespace);
    executor.add_node(node); executor.add_node(client_node);
  }
  void state(bool ready = true, int delta = 1, bool corrupt = false, float velocity = 0.f, int velocity_joint = 0) {
    tick += delta;
    State s{}; s.tick() = tick; s.mode_machine() = simulation_profile ? 0 : 6; s.mode_pr() = 0;
    s.imu_state().quaternion()[0] = 1;
    s.motor_state()[3].q() = ready ? .9f : 0;
    s.motor_state()[9].q() = ready ? .9f : 0;
    s.motor_state()[velocity_joint].dq() = velocity;
    s.crc() = crc32_core(reinterpret_cast<uint32_t*>(&s), (sizeof(s) >> 2) - 1);
    if (corrupt) s.crc() ^= 1;
    io->callback(s);
  }
  void trained_state(float velocity = 0.f, int joint = 0, float position_delta = 0.f, bool corrupt = false) {
    State s{}; s.tick() = ++tick; s.mode_machine() = 6; s.mode_pr() = 0;
    s.imu_state().quaternion()[0] = 1;
    auto defaults = h1_policy::to_sdk(h1_policy::defaults());
    for (int i = 0; i < 27; ++i) s.motor_state()[i].q() = defaults[i];
    s.motor_state()[joint].q() += position_delta; s.motor_state()[joint].dq() = velocity;
    s.crc() = crc32_core(reinterpret_cast<uint32_t*>(&s), (sizeof(s) >> 2) - 1);
    if (corrupt) s.crc() ^= 1;
    io->callback(s);
  }
  Trigger::Response call(const char* name) {
    auto c = client_node->create_client<Trigger>(test_namespace + "/h1_2_policy_node/" + name);
    CHECK(c->wait_for_service(2s));
    auto f = c->async_send_request(std::make_shared<Trigger::Request>());
    CHECK(executor.spin_until_future_complete(f, 2s) == rclcpp::FutureReturnCode::SUCCESS);
    return *f.get();
  }
  void pump(int ms = 30) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) { executor.spin_some(); std::this_thread::sleep_for(1ms); }
  }
  std::string status() { auto r = call("status"); CHECK(r.success); return r.message; }
  void advance(double seconds, bool ready = true) { clock += seconds; state(ready); }
  template<class Message>
  typename rclcpp::Publisher<Message>::SharedPtr publisher(const std::string& topic) {
    return client_node->create_publisher<Message>(topic, 10);
  }
  bool claim(const std::string& owner, const std::vector<uint8_t>& cmd_gid, const std::vector<uint8_t>& heartbeat_gid) {
    auto client = client_node->create_client<ClaimInput>(test_namespace + "/h1_2_policy_node/claim_input");
    CHECK(client->wait_for_service(2s));
    auto request = std::make_shared<ClaimInput::Request>();
    request->owner_boot_id = owner;
    request->cmd_writer_gid = cmd_gid;
    request->heartbeat_writer_gid = heartbeat_gid;
    auto future = client->async_send_request(request);
    CHECK(executor.spin_until_future_complete(future, 2s) == rclcpp::FutureReturnCode::SUCCESS);
    return future.get()->success;
  }
};

void disabled_and_shadow(const std::string& model) {
  Fixture f(model); f.state(); f.state();
  CHECK(f.status().find("input_identity_mode=legacy") != std::string::npos);
  CHECK(f.status().find("input_claimed=false") != std::string::npos);
  CHECK(f.status().find("owner_boot_id=none") != std::string::npos);
  CHECK(f.io->opens == 0); CHECK(!f.call("passive").success);
  CHECK(!f.node->set_parameter(rclcpp::Parameter("enable_hardware_output", true)).successful);
  CHECK(!f.node->set_parameter(rclcpp::Parameter("profile", "sim")).successful);
  CHECK(!f.call("authorize_output").success); CHECK(f.io->writes.empty());
  CHECK(f.call("shadow").success); f.pump(50);
  auto s = f.status(); CHECK(s.find("shadow=true") != std::string::npos);
  CHECK(s.find("publisher_present=false") != std::string::npos); CHECK(f.io->writes.empty());
  f.state(true, 1, true); CHECK(f.status().find("state=invalid") != std::string::npos);
  CHECK(!f.call("shadow").success); f.pump();
  CHECK(f.status().find("shadow=false") != std::string::npos); CHECK(f.io->writes.empty());
}

void authorized_budget(const std::string& model) {
  Fixture f(model, true); f.state(false); f.state(false);
  CHECK(f.call("authorize_output").success); CHECK(f.io->opens == 1);
  f.pump(25);
  CHECK(!f.io->writes.empty());
  const auto& damping = f.io->writes.back().motor_cmd();
  CHECK(damping[0].tau() == 0 && damping[0].kp() == 0);
  CHECK(std::abs(damping[0].kd() - 6.3f) < 1e-5f);
  CHECK(std::abs(damping[3].kd() - 10.1f) < 1e-5f);
  CHECK(!f.call("arm_hold").success);
  CHECK(f.call("fix_stand").success);
  f.advance(3.01, false); CHECK(f.call("arm_hold_stand").success);
  f.advance(3.01, false); CHECK(!f.call("arm_hold").success);
  f.state(); CHECK(f.call("arm_hold").success);
  size_t before = f.io->writes.size();
  f.pump(55); CHECK(f.io->writes.size() >= before + 2);
  bool nonzero = false;
  for (size_t i = before; i < before + 2; ++i)
    for (const auto& m : f.io->writes[i].motor_cmd()) nonzero |= std::abs(m.tau()) > 1e-6f;
  CHECK(nonzero); CHECK(f.status().find("mode=Passive") != std::string::npos);
  CHECK(!f.call("arm_hold").success);
  CHECK(f.call("revoke_output").success); CHECK(!f.io->present);
  CHECK(!f.call("fix_stand").success);
}

void fault_case(const std::string& model, int kind) {
  Fixture f(model, true); f.state(); f.state(); CHECK(f.call("authorize_output").success);
  f.pump(); size_t before = f.io->writes.size();
  if (kind == 0) f.state(true, 1, true);
  if (kind == 1) f.state(true, -1);
  if (kind == 2) { f.clock += .11; f.state(true, 0); }
  CHECK(f.status().find("state=invalid") != std::string::npos);
  CHECK(!f.call("fix_stand").success); f.pump();
  CHECK(!f.io->present); CHECK(f.io->writes.size() == before);
  f.state(); f.pump(); CHECK(!f.io->present); CHECK(f.io->writes.size() == before);
  CHECK(!f.call("fix_stand").success);
}

void slow_open(const std::string& model) {
  // Opening DDS may block for 100 ms. A pre-open sample cannot authorize output.
  Fixture f(model, true); f.state(); f.state();
  f.io->on_open = [&] { f.clock += .11; };
  CHECK(!f.call("authorize_output").success);
  CHECK(!f.io->present); CHECK(f.io->writes.empty());
  CHECK(f.status().find("authorized=false") != std::string::npos);
}

void slow_open_fresh_state(const std::string& model) {
  Fixture f(model, true); f.state(); f.state();
  // A synchronous callback also proves the state mutex is not held by open.
  f.io->on_open = [&] { f.clock += .11; f.state(); };
  CHECK(f.call("authorize_output").success); f.pump();
  CHECK(!f.io->writes.empty()); CHECK(f.call("revoke_output").success);
}

void simulation(const std::string& model) {
  Fixture f(model, false, true); f.state(); CHECK(f.io->present);
  CHECK(f.call("passive").success); f.pump(); CHECK(!f.io->writes.empty());
  CHECK(f.io->writes.back().motor_cmd()[0].kd() == 0);
}

void position_pd_simulation(const std::string& model) {
  Fixture f(model, false, true, "position_pd"); f.state();
  CHECK(f.status().find("joint_command_mode=position_pd") != std::string::npos);
  CHECK(!f.node->set_parameter(rclcpp::Parameter("joint_command_mode", "torque")).successful);
  CHECK(f.call("passive").success); f.pump();
  CHECK(!f.io->writes.empty());
  auto damping = h1_policy::to_sdk(h1_policy::passive_damping_gains());
  for (int i = 0; i < 27; ++i) {
    const auto& motor = f.io->writes.back().motor_cmd()[i];
    CHECK(motor.mode() == 1 && motor.tau() == 0 && motor.dq() == 0);
    CHECK(motor.kp() == 0 && motor.kd() == damping[i]);
  }
  CHECK(f.call("fix_stand").success);
  f.advance(3.01); CHECK(f.call("arm_hold_stand").success);
  f.advance(3.01); f.pump();
  CHECK(!f.io->writes.empty());
  const auto& cmd = f.io->writes.back();
  CHECK(h1_policy::valid_crc(cmd, cmd.crc()));
  for (int i = 0; i < 27; ++i) {
    const auto& motor = cmd.motor_cmd()[i];
    CHECK(motor.mode() == 1 && motor.tau() == 0 && motor.dq() == 0);
    CHECK(motor.kp() > 0 && motor.kd() > 0);
  }
  CHECK(std::abs(cmd.motor_cmd()[17].q() - (-.030f)) < 1e-5f);
  CHECK(std::abs(cmd.motor_cmd()[18].q() - (-.003f)) < 1e-5f);
  CHECK(std::abs(cmd.motor_cmd()[24].q() - (-.076f)) < 1e-5f);
  CHECK(std::abs(cmd.motor_cmd()[25].q() - .004f) < 1e-5f);
  CHECK(f.call("arm_hold").success);
  size_t first_policy_write = f.io->writes.size();
  f.pump(); CHECK(f.io->writes.size() > first_policy_write);
  h1_policy::State state{}; state.q[3] = state.q[9] = .9f; state.gravity[2] = -1;
  h1_policy::Core core;
  h1_policy::OrtPolicy policy(model);
  auto expected = core.control(state, policy.infer(core.observe(state, {0, 0, 0})));
  auto expected_q = h1_policy::to_sdk(expected.q);
  auto expected_kp = h1_policy::to_sdk(expected.kp);
  auto expected_kd = h1_policy::to_sdk(expected.kd);
  const auto& policy_cmd = f.io->writes[first_policy_write];
  CHECK(h1_policy::valid_crc(policy_cmd, policy_cmd.crc()));
  for (int i = 0; i < 27; ++i) {
    const auto& motor = policy_cmd.motor_cmd()[i];
    CHECK(std::abs(motor.q() - expected_q[i]) < 1e-5f);
    CHECK(motor.kp() == expected_kp[i] && motor.kd() == expected_kd[i]);
    CHECK(motor.tau() == 0 && motor.dq() == 0);
  }
  size_t before = f.io->writes.size();
  f.clock += .11; f.pump(); CHECK(f.io->writes.size() == before);
}

void reject_command_modes(const std::string& model) {
  bool rejected = false;
  try { Fixture f(model, false, true, "invalid"); } catch (const std::runtime_error&) { rejected = true; }
  CHECK(rejected);
}

void position_pd_hardware(const std::string& model) {
  {
    Fixture f(model, false, false, "position_pd"); f.state(); f.state();
    CHECK(!f.call("authorize_output").success); CHECK(!f.io->present);
    CHECK(!f.call("fix_stand").success); f.pump(); CHECK(f.io->writes.empty());
  }
  {
    Fixture f(model, true, false, "position_pd", 0); f.state(); f.state();
    CHECK(f.status().find("software_torque_clamp=false") != std::string::npos);
    CHECK(!f.node->set_parameter(rclcpp::Parameter("joint_command_mode", "torque")).successful);
    CHECK(f.call("authorize_output").success); f.pump(); CHECK(!f.io->writes.empty());
    auto damping = h1_policy::to_sdk(h1_policy::passive_damping_gains());
    for (int i = 0; i < 27; ++i) {
      const auto& m = f.io->writes.back().motor_cmd()[i];
      CHECK(m.q() == 0 && m.dq() == 0 && m.kp() == 0 && m.tau() == 0 && m.kd() == damping[i]);
    }
    CHECK(h1_policy::valid_crc(f.io->writes.back(), f.io->writes.back().crc()));
    CHECK(f.call("fix_stand").success); f.pump(); f.advance(3.01); f.pump(); CHECK(f.call("arm_hold_stand").success);
    f.pump(); f.advance(3.01); f.pump(); const auto cmd = f.io->writes.back();
    CHECK(h1_policy::valid_crc(cmd, cmd.crc()));
    CHECK(std::abs(cmd.motor_cmd()[17].q() - (-.030f)) < 1e-5f);
    CHECK(std::abs(cmd.motor_cmd()[18].q() - (-.003f)) < 1e-5f);
    for (int i = 0; i < 27; ++i) {
      const auto& m = cmd.motor_cmd()[i];
      CHECK(m.mode() == 1 && m.kp() > 0 && m.kd() > 0 && m.dq() == 0 && m.tau() == 0);
    }
    CHECK(!f.call("arm_hold").success);
    CHECK(f.call("revoke_output").success); auto before = f.io->writes.size();
    f.pump(); CHECK(!f.io->present && f.io->writes.size() == before);
  }
  {
    Fixture f(model, true, false, "position_pd", 0); f.state(); f.state();
    CHECK(f.call("authorize_output").success); f.pump();
    auto before = f.io->writes.size(); f.state(true, 1, true); f.pump();
    CHECK(!f.io->present && f.io->writes.size() == before);
  }
}

void position_pd_hardware_single_policy_step(const std::string& model) {
  Fixture f(model, true, false, "position_pd", 1); f.state(); f.state();
  CHECK(f.call("authorize_output").success); f.pump(); CHECK(f.call("fix_stand").success);
  f.pump(); f.advance(3.01); f.pump(); CHECK(f.call("arm_hold_stand").success);
  f.pump(); f.advance(3.01); f.pump(); auto before = f.io->writes.back();
  CHECK(f.call("arm_hold").success); f.pump(25);
  CHECK(f.status().find("mode=Hold") != std::string::npos);
  CHECK(f.status().find("controlled_stop_supported=false") != std::string::npos);
  CHECK(f.status().find("exit_protocol=hold_supported_release_v3") != std::string::npos);
  auto first = f.io->writes.back();
  for (int i=0;i<27;++i) {
    CHECK(std::abs(first.motor_cmd()[i].kp()-before.motor_cmd()[i].kp()) < 1e-5f);
    CHECK(std::abs(first.motor_cmd()[i].kd()-before.motor_cmd()[i].kd()) < 1e-5f);
    CHECK(std::abs(first.motor_cmd()[i].q()-before.motor_cmd()[i].q()) < 1e-5f);
  }
  CHECK(!f.call("arm_hold").success); CHECK(f.call("passive").success);
  f.advance(10.0); f.pump(25); CHECK(f.status().find("output_transition=hold") != std::string::npos);
  auto held=f.io->writes.back();
  for (int i=0;i<27;++i) { CHECK(held.motor_cmd()[i].kp()==first.motor_cmd()[i].kp()); CHECK(held.motor_cmd()[i].kd()==first.motor_cmd()[i].kd()); CHECK(std::abs(held.motor_cmd()[i].q()-first.motor_cmd()[i].q())<1e-5f); }
  CHECK(!f.call("fix_stand").success); CHECK(f.call("passive").success);
  CHECK(f.call("release_supported").success); f.pump(25);
  CHECK(f.status().find("output_transition=exit") != std::string::npos);
  CHECK(!f.call("release_supported").success);
  f.advance(1.5); f.pump(25); auto middle=f.io->writes.back();
  CHECK(middle.motor_cmd()[0].kp() < first.motor_cmd()[0].kp());
  CHECK(middle.motor_cmd()[0].kd() >= first.motor_cmd()[0].kd());
  f.advance(1.51); f.pump(25); CHECK(f.status().find("release_phase=settling") != std::string::npos);
  f.advance(.51); f.pump(25); CHECK(f.status().find("release_phase=damping_down") != std::string::npos);
  f.advance(3.01); f.pump(25); CHECK(f.status().find("mode=Passive") != std::string::npos);
  CHECK(f.status().find("output_transition=none") != std::string::npos);
  auto end=f.io->writes.back(); CHECK(end.motor_cmd()[0].kp()==0);
  CHECK(h1_policy::valid_crc(end,end.crc()));
  CHECK(f.call("revoke_output").success); auto n=f.io->writes.size(); f.pump(); CHECK(f.io->writes.size()==n);
}

float status_number(const std::string& status, const std::string& key) {
  const auto pos = status.find(key + "=");
  CHECK(pos != std::string::npos);
  return std::stof(status.substr(pos + key.size() + 1));
}
void check_block_command(const std::string& status, const Command& previous, int policy_index, int sdk_index) {
  CHECK(status_number(status, "last_release_block_policy_index") == policy_index);
  CHECK(std::abs(status_number(status, "last_release_block_cmd_q") - previous.motor_cmd()[sdk_index].q()) < 1e-5f);
  CHECK(std::abs(status_number(status, "last_release_block_cmd_kp") - previous.motor_cmd()[sdk_index].kp()) < 1e-5f);
  CHECK(std::abs(status_number(status, "last_release_block_cmd_kd") - previous.motor_cmd()[sdk_index].kd()) < 1e-5f);
}
void position_pd_release_blocked(const std::string& model, bool after_unloading) {
  Fixture f(model,true,false,"position_pd",0); f.state(); f.state();
  CHECK(f.call("authorize_output").success); f.pump();
  CHECK(f.call("fix_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("arm_hold_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("passive").success); f.pump();
  CHECK(f.call("release_supported").success); f.pump();
  if (after_unloading) {
    f.advance(3.01); f.pump();
    f.advance(.51); f.pump();
    CHECK(f.status().find("release_phase=damping_down") != std::string::npos);
    f.advance(1.0); f.pump();
  } else {
    f.advance(1.0); f.pump();
  }
  auto previous = f.io->writes.back();
  f.state(true, 1, false, .5f); f.pump();
  auto status = f.status();
  CHECK(status.find("mode=Hold") != std::string::npos);
  CHECK(status.find("release_blocked=true") != std::string::npos);
  CHECK(status.find("last_release_block_reason=speed last_release_block_source=snapshot") != std::string::npos);
  check_block_command(status, previous, 0, 0);
  CHECK(status.find(after_unloading ? "release_kp_zero=true" : "release_kp_zero=false") != std::string::npos);
  auto held = f.io->writes.back();
  CHECK(held.motor_cmd()[0].kp() == previous.motor_cmd()[0].kp());
  CHECK(held.motor_cmd()[0].kd() == previous.motor_cmd()[0].kd());
  CHECK(!f.call("release_supported").success);
  CHECK(f.call("passive").success);
  CHECK(f.status().find("release_blocked=true") != std::string::npos);
  CHECK(f.call("revoke_output").success);
  auto count = f.io->writes.size(); f.pump();
  CHECK(!f.io->present && f.io->writes.size() == count);
  CHECK(f.status().find("last_release_block_reason=speed") != std::string::npos);
}

void position_pd_release_newer_nonquiet_sample(const std::string& model, bool at_completion, float velocity = .2f) {
  Fixture f(model,true,false,"position_pd",0); f.state(); f.state();
  CHECK(f.call("authorize_output").success); f.pump();
  CHECK(f.call("fix_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("arm_hold_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("passive").success); f.pump();
  CHECK(f.call("release_supported").success); f.pump();
  f.advance(3.01); f.pump(); f.advance(.51); f.pump();
  CHECK(f.status().find("release_phase=damping_down") != std::string::npos);
  const auto previous = f.io->writes.back();
  CHECK(previous.motor_cmd()[16].kd() > .5f);
  // read_locked consumes three clock reads and the velocity watchdog two;
  // inject a newer sample on the sixth read, just after the initial state
  // snapshot and before the guarded release candidate is calculated.
  f.advance(at_completion ? 3.01 : 1.0);
  f.clock_reads = 0;
  f.inject_on_clock_read = 6;
  f.clock_injection = [&f, velocity] { f.state(true, 1, false, velocity, 16); };
  f.pump();
  CHECK(!f.clock_injection);
  const auto status = f.status();
  if (velocity >= h1_policy::kReleaseBlockSpeed) {
    CHECK(status.find("mode=Hold") != std::string::npos);
    CHECK(status.find("release_blocked=true") != std::string::npos);
    CHECK(status.find("release_kp_zero=true") != std::string::npos);
    CHECK(status.find("last_release_block_reason=speed last_release_block_source=newer_state") != std::string::npos);
    check_block_command(status, previous, 16, 16);
  } else {
    CHECK(status.find("mode=Stopping") != std::string::npos);
    CHECK(status.find("release_phase=settling") != std::string::npos);
    CHECK(status.find("release_blocked=false") != std::string::npos);
  }
  CHECK(f.io->writes.back().motor_cmd()[16].kd() == previous.motor_cmd()[16].kd());
  CHECK(f.call("revoke_output").success);
}

void position_pd_release_deadline_after_calculation(const std::string& model) {
  Fixture f(model,true,false,"position_pd",0); f.state(); f.state();
  CHECK(f.call("authorize_output").success); f.pump();
  CHECK(f.call("fix_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("arm_hold_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("passive").success); f.pump();
  CHECK(f.call("release_supported").success); f.pump();
  f.advance(3.01); f.pump(); f.advance(.51); f.pump();
  CHECK(f.status().find("release_phase=damping_down") != std::string::npos);
  const auto previous = f.io->writes.back();
  f.advance(6.475);  // Still before the 10 s total deadline.
  f.clock_reads = 0; f.jump_on_clock_read = 8; f.clock_jump = .012;
  f.pump();
  const auto status = f.status();
  CHECK(status.find("mode=Hold") != std::string::npos);
  CHECK(status.find("release_blocked=true") != std::string::npos);
  CHECK(status.find("last_release_block_reason=timeout last_release_block_source=newer_state") != std::string::npos);
  check_block_command(status, previous, 0, 0);
  CHECK(f.io->writes.back().motor_cmd()[16].kd() == previous.motor_cmd()[16].kd());
  CHECK(f.call("revoke_output").success);
}

void position_pd_entry_blending(const std::string& model) {
  Fixture f(model,true,false,"position_pd",500); f.state(); f.state(); CHECK(!f.call("release_supported").success); CHECK(f.call("authorize_output").success);
  CHECK(!f.call("release_supported").success); f.pump();
  auto passive=f.io->writes.back(); CHECK(f.call("fix_stand").success); f.pump();
  auto first=f.io->writes.back();
  for(int i=0;i<27;++i) { CHECK(first.motor_cmd()[i].kp()==passive.motor_cmd()[i].kp()); CHECK(first.motor_cmd()[i].kd()==passive.motor_cmd()[i].kd()); }
  CHECK(!f.call("arm_hold_stand").success);
  f.advance(1.5); f.pump(); CHECK(f.io->writes.back().motor_cmd()[0].kp()>0);
  f.advance(1.51); f.pump(); CHECK(f.status().find("output_transition=none") != std::string::npos);
  auto prep=f.io->writes.back(); CHECK(std::abs(prep.motor_cmd()[0].kp()-98.7f)<1e-4f);
  CHECK(f.call("arm_hold_stand").success); f.pump(); auto first_stand=f.io->writes.back();
  for(int i=0;i<27;++i) { CHECK(std::abs(first_stand.motor_cmd()[i].kp()-prep.motor_cmd()[i].kp())<1e-4f); CHECK(std::abs(first_stand.motor_cmd()[i].kd()-prep.motor_cmd()[i].kd())<1e-4f); }
  f.advance(3.01); f.pump(); CHECK(f.status().find("output_transition=none") != std::string::npos);
  auto stand=f.io->writes.back(); CHECK(std::abs(stand.motor_cmd()[16].kp()-120.f)<1e-4f);
  CHECK(f.call("arm_hold").success); f.pump(); auto first_policy=f.io->writes.back();
  for(int i=0;i<27;++i) { CHECK(std::abs(first_policy.motor_cmd()[i].kp()-stand.motor_cmd()[i].kp())<1e-4f); CHECK(std::abs(first_policy.motor_cmd()[i].kd()-stand.motor_cmd()[i].kd())<1e-4f); }
  f.advance(1.5); f.pump(); CHECK(f.io->writes.back().motor_cmd()[0].kp()>stand.motor_cmd()[0].kp());
  f.advance(1.51); f.pump(); CHECK(f.status().find("output_transition=none") != std::string::npos);
  CHECK(std::abs(f.io->writes.back().motor_cmd()[0].kp()-150.f)<1e-4f);
  CHECK(f.call("revoke_output").success);
}

void position_pd_exit_faults(const std::string& model) {
  for(int kind=0;kind<3;++kind) {
    Fixture f(model,true,false,"position_pd",1); f.state(); f.state(); CHECK(f.call("authorize_output").success); f.pump();
    CHECK(f.call("fix_stand").success); f.pump(); CHECK(f.call("passive").success); f.pump();
    CHECK(f.status().find("mode=Hold") != std::string::npos);
    auto n=f.io->writes.size();
    if(kind==0) { CHECK(f.call("revoke_output").success); n=f.io->writes.size(); }
    if(kind==1) { f.clock+=.11; }
    if(kind==2) f.io->fail_write=true;
    f.pump(); CHECK(!f.io->present); CHECK(f.io->writes.size()==n);
  }
}


void continuous_navigation_commands(const std::string& model) {
  Fixture f(model, true, false, "position_pd", 0, true, true); f.state(); f.state();
  auto pub = f.client_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_final", 10);
  auto heartbeat = f.client_node->create_publisher<std_msgs::msg::Bool>("/h1_2/policy_gate_alive", 10);
  std_msgs::msg::Bool alive; alive.data = true;
  geometry_msgs::msg::Twist v; v.linear.x = .18; v.angular.z = .3;
  CHECK(f.call("authorize_output").success); f.pump(); CHECK(f.call("fix_stand").success);
  f.pump(); f.advance(3.01); f.pump(); CHECK(f.call("arm_hold_stand").success);
  f.pump(); f.advance(3.01); f.pump();
  pub->publish(v); f.pump(); CHECK(f.call("arm_hold").success);
  f.pump(); CHECK(f.status().find("navigation_ready=false") != std::string::npos);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);
  f.advance(3.01); heartbeat->publish(alive); f.pump(70); CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);
  f.advance(.01); pub->publish(v); f.pump(70);
  CHECK(f.status().find("policy_cmd=0.18,0,0.3") != std::string::npos);
  f.advance(.21); f.pump(); CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);
  CHECK(f.status().find("navigation_ready=false") != std::string::npos);
  heartbeat->publish(alive); f.pump(70); CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  f.advance(.41); heartbeat->publish(alive); f.pump(70); CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);
  CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  CHECK(f.call("revoke_output").success); CHECK(f.status().find("navigation_ready=false") != std::string::npos);
}

void claimed_input_validation(const std::string& model) {
  const std::string owner(32, 'a');
  Fixture f(model, false, false, "torque", 0, false, false, 0, false, true, owner);
  auto cmd = f.publisher<geometry_msgs::msg::Twist>("/cmd_vel_final");
  auto heartbeat = f.publisher<std_msgs::msg::Bool>("/h1_2/policy_gate_alive");
  const auto cmd_gid = publisher_gid<geometry_msgs::msg::Twist>(cmd);
  const auto heartbeat_gid = publisher_gid<std_msgs::msg::Bool>(heartbeat);
  CHECK(cmd_gid.size() == RMW_GID_STORAGE_SIZE && heartbeat_gid.size() == RMW_GID_STORAGE_SIZE);
  CHECK(h1_policy::nonzero_ros_gid(cmd_gid) && h1_policy::nonzero_ros_gid(heartbeat_gid));
  CHECK(!f.claim(std::string(32, 'b'), cmd_gid, heartbeat_gid));
  for (size_t length : {size_t(1), size_t(16), size_t(23), size_t(25)}) {
    auto malformed = cmd_gid; malformed.resize(length);
    CHECK(!f.claim(owner, malformed, heartbeat_gid));
  }
  auto zero = std::vector<uint8_t>(RMW_GID_STORAGE_SIZE, 0);
  CHECK(!f.claim(owner, zero, heartbeat_gid));
  CHECK(!f.claim(owner, cmd_gid, zero));
  auto status = f.status();
  CHECK(status.find("input_identity_mode=claimed") != std::string::npos);
  CHECK(status.find("input_claimed=false") != std::string::npos);
  CHECK(status.find("cmd_writer_gid=none") != std::string::npos);
  CHECK(status.find("heartbeat_writer_gid=none") != std::string::npos);
  CHECK(status.find("authorized=false") != std::string::npos);
  CHECK(!f.io->present && f.io->writes.empty());
  CHECK(f.claim(owner, cmd_gid, heartbeat_gid));
  status = f.status();
  CHECK(status.find("input_claimed=true") != std::string::npos);
  CHECK(status.find("owner_boot_id=" + owner) != std::string::npos);
  CHECK(status.find("cmd_writer_gid=" + gid_hex(cmd_gid)) != std::string::npos);
  CHECK(status.find("heartbeat_writer_gid=" + gid_hex(heartbeat_gid)) != std::string::npos);
  CHECK(status.find("authorized=false") != std::string::npos);
  CHECK(!f.io->present && f.io->writes.empty());
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> expected{};
  std::copy(cmd_gid.begin(), cmd_gid.end(), expected.begin());
  rmw_gid_t actual{};
  std::copy(cmd_gid.begin(), cmd_gid.end(), actual.data);
  CHECK(h1_policy::matches_ros_gid(expected, actual));
  actual.data[16] ^= 1;
  CHECK(!h1_policy::matches_ros_gid(expected, actual));
  CHECK(!f.claim(std::string(32, 'b'), cmd_gid, heartbeat_gid));
  status = f.status();
  CHECK(status.find("input_claimed=false") != std::string::npos);
  CHECK(status.find("cmd_writer_gid=none") != std::string::npos);
  CHECK(!f.io->present && f.io->writes.empty());
}

void reject_invalid_claimed_startup_identity(const std::string& model) {
  bool rejected = false;
  try { Fixture f(model, false, false, "torque", 0, false, false, 0, false, true, ""); }
  catch (const std::runtime_error&) { rejected = true; }
  CHECK(rejected);
  rejected = false;
  try { Fixture f(model, false, false, "torque", 0, false, false, 0, false, true, std::string(31, 'a') + "A"); }
  catch (const std::runtime_error&) { rejected = true; }
  CHECK(rejected);
}

void claimed_input_replacement_clears_and_filters(const std::string& model) {
  const std::string owner(32, 'c');
  Fixture f(model, false, true, "torque", 0, true, false, 0, false, true, owner);
  f.state();
  auto old_cmd = f.publisher<geometry_msgs::msg::Twist>("/cmd_vel_final");
  auto old_heartbeat = f.publisher<std_msgs::msg::Bool>("/h1_2/policy_gate_alive");
  auto old_cmd_gid = publisher_gid<geometry_msgs::msg::Twist>(old_cmd);
  auto old_heartbeat_gid = publisher_gid<std_msgs::msg::Bool>(old_heartbeat);
  geometry_msgs::msg::Twist nonzero; nonzero.linear.x = .12;
  std_msgs::msg::Bool alive; alive.data = true;
  old_cmd->publish(nonzero); old_heartbeat->publish(alive); f.pump();
  CHECK(f.call("passive").success); CHECK(f.call("fix_stand").success);
  f.advance(3.01); CHECK(f.call("arm_hold_stand").success);
  f.advance(3.01); CHECK(f.call("arm_hold").success); f.pump(40);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);

  CHECK(f.claim(owner, old_cmd_gid, old_heartbeat_gid));
  CHECK(f.status().find("navigation_ready=false") != std::string::npos);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);
  old_heartbeat->publish(alive); old_cmd->publish(nonzero); f.pump(50);
  CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  f.advance(.01);
  old_cmd->publish(nonzero); f.pump(40);
  CHECK(f.status().find("policy_cmd=0.12,0,0") != std::string::npos);

  auto new_cmd = f.publisher<geometry_msgs::msg::Twist>("/cmd_vel_final");
  auto new_heartbeat = f.publisher<std_msgs::msg::Bool>("/h1_2/policy_gate_alive");
  auto new_cmd_gid = publisher_gid<geometry_msgs::msg::Twist>(new_cmd);
  auto new_heartbeat_gid = publisher_gid<std_msgs::msg::Bool>(new_heartbeat);
  CHECK(new_cmd_gid != old_cmd_gid && new_heartbeat_gid != old_heartbeat_gid);
  CHECK(f.claim(owner, new_cmd_gid, new_heartbeat_gid));
  CHECK(f.status().find("navigation_ready=false") != std::string::npos);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);

  new_heartbeat->publish(alive); new_cmd->publish(geometry_msgs::msg::Twist{}); f.pump(50);
  CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  f.advance(.01); new_cmd->publish(nonzero); new_heartbeat->publish(alive); f.pump(40);
  CHECK(f.status().find("policy_cmd=0.12,0,0") != std::string::npos);
  CHECK(f.claim(owner, new_cmd_gid, new_heartbeat_gid));
  CHECK(f.status().find("navigation_ready=false") != std::string::npos);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);

  old_cmd->publish(nonzero); old_heartbeat->publish(alive); f.pump(60);
  CHECK(f.status().find("navigation_ready=false") != std::string::npos);
  CHECK(f.status().find("policy_cmd=0,0,0") != std::string::npos);
  new_heartbeat->publish(alive); new_cmd->publish(geometry_msgs::msg::Twist{}); f.pump(50);
  CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  f.advance(.01);
  new_cmd->publish(nonzero); new_heartbeat->publish(alive); f.pump(40);
  CHECK(f.status().find("policy_cmd=0.12,0,0") != std::string::npos);
  alive.data = false; old_heartbeat->publish(alive); f.pump(30);
  CHECK(f.status().find("navigation_ready=true") != std::string::npos);
  alive.data = true;
  f.advance(.21); old_heartbeat->publish(alive); f.pump(40);
  CHECK(f.status().find("navigation_ready=false") != std::string::npos);
}
void reject_continuous_misconfiguration(const std::string& model) {
  bool rejected = false; try { Fixture f(model, true, false, "torque", 0, true, true); } catch (const std::runtime_error&) { rejected = true; } CHECK(rejected);
  rejected = false; try { Fixture f(model, true, false, "position_pd", 0, false, true); } catch (const std::runtime_error&) { rejected = true; } CHECK(rejected);
  rejected = false; try { Fixture f(model, true, false, "position_pd", 5, true, true); } catch (const std::runtime_error&) { rejected = true; } CHECK(rejected);
}

void policy_deadline_before_and_during_step(const std::string& model, bool cross_during_step) {
  Fixture f(model, true, false, "position_pd", 155, false, false, 3.1);
  f.state(); f.state(); CHECK(f.call("authorize_output").success); f.pump();
  CHECK(f.call("fix_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("arm_hold_stand").success); f.pump(); f.advance(3.01); f.pump();
  CHECK(f.call("arm_hold").success); f.pump(25);
  CHECK(f.status().find("mode=ArmHold") != std::string::npos);
  auto last = f.io->writes.back();
  auto first_after = f.io->writes.size();
  if (cross_during_step) {
    // Fresh feedback remains valid while time passes across the deadline
    // during policy computation; the late result must never be written.
    f.advance(3.099);
    f.clock_reads = 0; f.clock_jump = .002; f.jump_on_clock_read = 8;
  } else {
    f.advance(3.101);
  }
  f.pump(70);
  CHECK(f.status().find("mode=Hold") != std::string::npos);
  CHECK(f.io->writes.size() > first_after);
  for (size_t n = first_after; n < f.io->writes.size(); ++n) {
    for (int i = 0; i < 27; ++i) {
      const auto& actual = f.io->writes[n].motor_cmd()[i];
      const auto& previous = last.motor_cmd()[i];
      CHECK(std::abs(actual.q() - previous.q()) < 1e-5f);
      CHECK(std::abs(actual.kp() - previous.kp()) < 1e-5f);
      CHECK(std::abs(actual.kd() - previous.kd()) < 1e-5f);
    }
  }
  CHECK(f.call("revoke_output").success);
}

void trained_guard_integration(const std::string& model) {
  Fixture f(model, true, false, "position_pd", 500, false, false, 0, true);
  f.trained_state(); f.trained_state(); CHECK(f.call("authorize_output").success); f.pump();
  CHECK(f.call("fix_stand").success); f.pump();
  f.clock += 3.01; f.trained_state(); f.pump(); CHECK(f.call("arm_hold_stand").success); f.pump();
  f.clock += 3.01; f.trained_state(); f.pump();
  CHECK(!f.io->writes.empty());
  const auto expected_stand = h1_policy::to_sdk(h1_policy::defaults());
  for (int i = 0; i < 27; ++i)
    CHECK(std::abs(f.io->writes.back().motor_cmd()[i].q() - expected_stand[i]) < 1e-5f);
  f.trained_state(0, 3, .27f);
  CHECK(f.status().find("entry_angle_gate=diagnostic_only") != std::string::npos);
  CHECK(f.call("arm_hold").success);
  CHECK(f.status().find("guard_prearm_sdk_index=3") != std::string::npos);
  CHECK(f.status().find("guard_prearm_max_delta=0.27") != std::string::npos);
  f.pump();
  // During the first two seconds the configured 1 rad/s limit is doubled.
  for (int i = 0; i < 2; ++i) { f.clock += .02; f.trained_state(2.1f, 17); f.pump(25); }
  CHECK(f.status().find("trained_guard_tripped=false") != std::string::npos);
  auto before = f.io->writes.size();
  f.clock += .02; f.trained_state(2.1f, 17); f.pump(25);
  CHECK(f.io->writes.size() > before);
  auto status = f.status();
  CHECK(status.find("mode=Passive") != std::string::npos);
  CHECK(status.find("trained_guard_tripped=true") != std::string::npos);
  CHECK(status.find("trained_guard_reason=joint_speed") != std::string::npos);
  CHECK(status.find("trained_guard_sdk_index=17") != std::string::npos);
  CHECK(status.find("trained_guard_native_damping=true") != std::string::npos);
  CHECK(status.find("authorized=true") != std::string::npos);
  auto damping = h1_policy::to_sdk(h1_policy::passive_damping_gains());
  for (size_t n = before; n < f.io->writes.size(); ++n)
    for (int i = 0; i < 27; ++i) {
      const auto& m = f.io->writes[n].motor_cmd()[i];
      CHECK(m.tau() == 0 && m.kp() == 0 && m.q() == 0);
      CHECK(std::abs(m.kd() - damping[i]) < 1e-5f);
    }
  CHECK(!f.call("fix_stand").success); CHECK(f.call("passive").success);
  f.clock += .02; f.trained_state(); f.pump(25);
  CHECK(f.status().find("trained_guard_tripped=true") != std::string::npos);
  f.trained_state(0, 0, 0, true); f.pump(25);
  CHECK(!f.io->present); CHECK(f.status().find("authorized=false") != std::string::npos);
  CHECK(f.status().find("trained_guard_tripped=true") != std::string::npos);
}


void trained_position_limit_integration(const std::string& model) {
  Fixture f(model, true, false, "position_pd", 500, false, false, 0, true);
  f.trained_state(); f.trained_state(); CHECK(f.call("authorize_output").success); f.pump();
  CHECK(f.call("fix_stand").success); f.pump();
  f.clock += 3.01; f.trained_state(); f.pump(); CHECK(f.call("arm_hold_stand").success); f.pump();
  f.clock += 3.01; f.trained_state(0, 3, .27f); f.pump(); CHECK(f.call("arm_hold").success);
  for (int i = 0; i < 3; ++i) { f.clock += .02; f.trained_state(0, 3, 2.2f); f.pump(25); }
  const auto status = f.status();
  CHECK(status.find("trained_guard_tripped=true") != std::string::npos);
  CHECK(status.find("trained_guard_reason=joint_position") != std::string::npos);
  CHECK(status.find("trained_guard_sdk_index=3") != std::string::npos);
  CHECK(status.find("trained_guard_native_damping=true") != std::string::npos);
}

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "usage: node_integration_test MODEL\n"; return 2; }
  rclcpp::init(argc, argv);
  try {
    disabled_and_shadow(argv[1]); authorized_budget(argv[1]);
    for (int kind = 0; kind < 3; ++kind) fault_case(argv[1], kind);
    slow_open(argv[1]); slow_open_fresh_state(argv[1]); simulation(argv[1]);
    position_pd_simulation(argv[1]); reject_command_modes(argv[1]); position_pd_hardware(argv[1]);
    position_pd_hardware_single_policy_step(argv[1]); position_pd_entry_blending(argv[1]); position_pd_exit_faults(argv[1]);
    position_pd_release_blocked(argv[1], false); position_pd_release_blocked(argv[1], true);
    position_pd_release_newer_nonquiet_sample(argv[1], false); position_pd_release_newer_nonquiet_sample(argv[1], true);
    position_pd_release_newer_nonquiet_sample(argv[1], true, .6f);
    position_pd_release_deadline_after_calculation(argv[1]);
    continuous_navigation_commands(argv[1]); reject_continuous_misconfiguration(argv[1]);
    policy_deadline_before_and_during_step(argv[1], false);
    policy_deadline_before_and_during_step(argv[1], true);
    claimed_input_validation(argv[1]); reject_invalid_claimed_startup_identity(argv[1]);
    claimed_input_replacement_clears_and_filters(argv[1]);
    trained_guard_integration(argv[1]); trained_position_limit_integration(argv[1]);
    rclcpp::shutdown(); std::cout << "node integration scenarios passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1; }
}
