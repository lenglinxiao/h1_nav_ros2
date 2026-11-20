// 独立的抓握后拉手势demo程序
// 用于挂吊架测试和示教，不依赖守护进程
// 编译: 在CMakeLists.txt中添加此目标

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "unitree/robot/channel/channel_factory.hpp"
#include "arm_gesture_player.hpp"
#include "grab_pull_config.hpp"

namespace {
volatile std::sig_atomic_t running = 1;

void handle_signal(int) {
  running = 0;
}
}  // namespace

void print_usage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " [options]\n"
            << "Options:\n"
            << "  --config <path>           Path to grab_pull.yaml config file\n"
            << "  --network-interface <if>  Network interface for DDS\n"
            << "  --domain-id <id>          DDS domain ID (default: 0)\n"
            << "  --no-play                 Load config and check, but don't play\n"
            << "  --help                    Show this help\n"
            << std::endl;
}

int main(int argc, char** argv) {
  std::string config_path;
  std::string network_interface;
  int domain_id = 0;
  bool no_play = false;

  // 解析参数
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << arg << std::endl;
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else if (arg == "--config") {
      const char* val = next();
      if (!val) return 2;
      config_path = val;
    } else if (arg == "--network-interface") {
      const char* val = next();
      if (!val) return 2;
      network_interface = val;
    } else if (arg == "--domain-id") {
      const char* val = next();
      if (!val) return 2;
      domain_id = std::atoi(val);
    } else if (arg == "--no-play") {
      no_play = true;
    } else {
      std::cerr << "Unknown option: " << arg << std::endl;
      print_usage(argv[0]);
      return 2;
    }
  }

  if (config_path.empty()) {
    std::cerr << "Error: --config is required\n" << std::endl;
    print_usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  // 加载配置
  std::cout << "Loading config from: " << config_path << std::endl;
  h1_2_grab_pull::GrabPullConfig config;
  std::string error;
  if (!h1_2_grab_pull::GrabPullConfig::load_from_file(config_path, config, error)) {
    std::cerr << "Failed to load config: " << error << std::endl;
    return 1;
  }
  std::cout << "Config loaded successfully" << std::endl;
  std::cout << "  Control period: " << config.trajectory_config.control_period << " s" << std::endl;
  std::cout << "  Max joint velocity: " << config.trajectory_config.max_joint_velocity << " rad/s" << std::endl;
  std::cout << "  Motion segments: " << config.motion_sequence.size() << std::endl;

  // 计算总时长
  h1_2_arm_trajectory::TrajectoryGenerator gen(config.trajectory_config);
  h1_2_arm_trajectory::ArmPose initial{0.28, 0.0, 0.0, 0.52, 0.0, 0.0, 0.0};  // 自然下垂
  double total_duration = gen.compute_total_duration(initial, config.motion_sequence);
  std::cout << "  Estimated total duration: " << total_duration << " s" << std::endl;

  if (no_play) {
    std::cout << "Config check complete (--no-play specified)" << std::endl;
    return 0;
  }

  // 初始化DDS
  std::cout << "\nInitializing DDS..." << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

  h1_2_arm_gesture::ArmGesturePlayer player;
  if (!player.init_dds(domain_id, network_interface)) {
    std::cerr << "Failed to initialize DDS: " << player.last_error() << std::endl;
    return 1;
  }
  std::cout << "DDS initialized" << std::endl;

  // 等待接收机器人状态
  std::cout << "\nWaiting for robot state..." << std::endl;
  if (!player.wait_for_state(5.0)) {
    std::cerr << "Timeout waiting for robot state: " << player.last_error() << std::endl;
    std::cerr << "Make sure the robot is powered on and connected" << std::endl;
    return 1;
  }
  std::cout << "Robot state received" << std::endl;

  std::cout << "\n========================================" << std::endl;
  std::cout << "Ready to play gesture" << std::endl;
  std::cout << "Press Ctrl+C at any time to abort" << std::endl;
  std::cout << "Starting in 3 seconds..." << std::endl;
  std::cout << "========================================\n" << std::endl;

  for (int i = 3; i > 0 && running; --i) {
    std::cout << i << "..." << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout << std::endl;

  if (!running) {
    std::cout << "Aborted by user" << std::endl;
    return 0;
  }

  // 播放手势
  std::cout << "Playing gesture..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();

  auto result = player.play_gesture(config.motion_sequence, config.trajectory_config);

  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

  std::cout << "\n========================================" << std::endl;
  if (result.success()) {
    std::cout << "✓ Gesture completed successfully" << std::endl;
    std::cout << "  Time elapsed: " << elapsed << " s" << std::endl;
  } else {
    std::cout << "✗ Gesture failed" << std::endl;
    std::cout << "  Error code: " << result.error_code << std::endl;
    std::cout << "  Message: " << result.message << std::endl;
    if (result.was_aborted) {
      std::cout << "  (Aborted by user)" << std::endl;
    }
  }
  std::cout << "========================================" << std::endl;

  return result.success() ? 0 : 1;
}
