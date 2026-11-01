#include "h1_2_policy_bridge/node.hpp"
#include "check.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <unistd.h>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using State = unitree_hg::msg::dds_::LowState_;
using Command = unitree_hg::msg::dds_::LowCmd_;

struct FakeIo final : h1_policy::LowLevelIo {
  StateCallback callback;
  bool present = false;
  int opens = 0, closes = 0;
  std::vector<Command> writes;
  void start(int, const std::string&, StateCallback cb) override { callback = std::move(cb); }
  void open_output() override { present = true; ++opens; }
  bool write(const Command& c) override { CHECK(present); writes.push_back(c); return true; }
  void close_output() override { present = false; ++closes; }
  bool output_present() const override { return present; }
  void stop() override { callback = {}; }
};

struct Fixture {
  double clock = 10;
  uint32_t tick = 0;
  bool simulation_profile;
  const std::string test_namespace = "/h1_policy_test_" + std::to_string(getpid());
  FakeIo* io;
  std::shared_ptr<h1_policy::PolicyNode> node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  Fixture(const std::string& model, bool enabled = false, bool sim = false) {
    simulation_profile = sim;
    auto fake = std::make_unique<FakeIo>(); io = fake.get();
    rclcpp::NodeOptions options;
    // Service calls must never resolve to a live robot's policy node, even
    // when this binary is invoked outside the CTest environment.
    options.arguments({"--ros-args", "-r", "__ns:=" + test_namespace});
    options.parameter_overrides({rclcpp::Parameter("model_path", model),
      rclcpp::Parameter("profile", sim ? "sim" : "hardware"),
      rclcpp::Parameter("network_interface", sim ? "lo" : "eth0"),
      rclcpp::Parameter("dds_domain", sim ? 42 : 0),
      rclcpp::Parameter("enable_hardware_output", enabled),
      rclcpp::Parameter("operator_confirmed_debug_mode", enabled),
      rclcpp::Parameter("operator_confirmed_exclusive_control", enabled),
      rclcpp::Parameter("max_policy_steps", enabled ? 2 : 0)});
    node = std::make_shared<h1_policy::PolicyNode>(options, std::move(fake), [this] { return clock; });
    client_node = std::make_shared<rclcpp::Node>("integration_client", test_namespace);
    executor.add_node(node); executor.add_node(client_node);
  }
  void state(bool ready = true, int delta = 1, bool corrupt = false) {
    tick += delta;
    State s{}; s.tick() = tick; s.mode_machine() = simulation_profile ? 0 : 6; s.mode_pr() = 0;
    s.imu_state().quaternion()[0] = 1;
    s.motor_state()[3].q() = ready ? .9f : 0;
    s.motor_state()[9].q() = ready ? .9f : 0;
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
};

void disabled_and_shadow(const std::string& model) {
  Fixture f(model); f.state(); f.state();
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

void simulation(const std::string& model) {
  Fixture f(model, false, true); f.state(); CHECK(f.io->present);
  CHECK(f.call("passive").success); f.pump(); CHECK(!f.io->writes.empty());
  CHECK(f.io->writes.back().motor_cmd()[0].kd() == 0);
}

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "usage: node_integration_test MODEL\n"; return 2; }
  rclcpp::init(argc, argv);
  try {
    disabled_and_shadow(argv[1]); authorized_budget(argv[1]);
    for (int kind = 0; kind < 3; ++kind) fault_case(argv[1], kind);
    simulation(argv[1]);
    rclcpp::shutdown(); std::cout << "6 node integration scenarios passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1; }
}
