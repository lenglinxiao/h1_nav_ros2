#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <unitree/dds_wrapper/common/crc.h>
#include <chrono>
#include <thread>
#include <iostream>
using namespace std::chrono_literals;
using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
int main(int argc, char **argv) {
  bool poison = argc > 1 && std::string(argv[1]) == "nonfinite";
  unitree::robot::ChannelFactory::Instance()->Init(42, "lo");
  auto pub = std::make_shared<unitree::robot::ChannelPublisher<LS>>("rt/lowstate");
  pub->InitChannel();
  std::atomic<int> count{0};
  std::atomic<bool> valid{true};
  auto sub = std::make_shared<unitree::robot::ChannelSubscriber<LC>>("rt/lowcmd");
  sub->InitChannel([&](const void *data) {
    auto message = *static_cast<const LC *>(data);
    auto crc = crc32_core(reinterpret_cast<uint32_t *>(&message), (sizeof(message) >> 2) - 1);
    if (crc != message.crc()) valid = false;
    for (int i = 0; i < 27; ++i) {
      auto motor = message.motor_cmd()[i];
      if (!std::isfinite(motor.tau()) || std::abs(motor.tau()) > 300 || motor.kp() != 0 || motor.kd() != 0) valid = false;
    }
    ++count;
  });
  LS state{};
  state.imu_state().quaternion()[0] = 1;
  std::atomic<bool> publish{true};
  std::atomic<bool> bad{false};
  std::thread thread([&] { while (publish) { if (bad) state.motor_state()[0].q() = std::numeric_limits<float>::quiet_NaN(); pub->Write(state, 0); std::this_thread::sleep_for(20ms); } });
  std::this_thread::sleep_for(2s);
  bool ok = count == 0;
  std::cout << "startup count=" << count.load() << std::endl;
  std::this_thread::sleep_for(4s);
  if (count < 5) ok = false;
  if (poison) bad = true;
  else publish = false;
  std::this_thread::sleep_for(400ms);
  int stopped = count;
  std::this_thread::sleep_for(300ms);
  if (count != stopped || !valid) ok = false;
  publish = false;
  thread.join();
  std::cout << "startup quiet, manual passive, fault stop, CRC/torque: " << (ok ? "pass" : "FAIL") << " (commands=" << count.load() << ")\n";
  return ok ? 0 : 1;
}
