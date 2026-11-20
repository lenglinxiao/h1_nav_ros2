#include <arpa/inet.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/h1/loco/h1_loco_client.hpp"
#include "arm_gesture_player.hpp"
#include "grab_pull_config.hpp"

namespace {
volatile std::sig_atomic_t running = 1;

// 手势播放状态
struct GesturePlaybackState {
  std::atomic<bool> is_playing{false};
  std::atomic<bool> abort_requested{false};
  std::mutex mutex;
  std::condition_variable cv;
  std::thread worker_thread;

  int result_code = 0;
  std::string result_message;
  bool result_ready = false;

  sockaddr_in client_addr{};
  socklen_t client_addr_len = 0;
};

GesturePlaybackState gesture_state;
std::unique_ptr<h1_2_arm_gesture::ArmGesturePlayer> arm_player;
std::string grab_pull_config_path;

void handle_signal(int) {
  running = 0;
  gesture_state.abort_requested.store(true);
}

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void send_response(int sock, const sockaddr_in& peer, socklen_t peer_len, const std::string& response) {
  sendto(sock, response.data(), response.size(), 0,
         reinterpret_cast<const sockaddr*>(&peer), peer_len);
}

std::string rest_of_line(std::istringstream& in) {
  std::string rest;
  std::getline(in, rest);
  if (!rest.empty() && rest.front() == ' ') {
    rest.erase(0, 1);
  }
  return rest;
}

// 工作线程函数
void gesture_worker_thread(int sock) {
  using namespace h1_2_grab_pull;

  // 加载配置
  GrabPullConfig config;
  std::string error;
  if (!GrabPullConfig::load_from_file(grab_pull_config_path, config, error)) {
    std::lock_guard<std::mutex> lock(gesture_state.mutex);
    gesture_state.result_code = -5;
    gesture_state.result_message = error;
    gesture_state.result_ready = true;
    gesture_state.cv.notify_one();
    return;
  }

  // 添加"收回"段
  h1_2_arm_trajectory::MotionSegment return_seg;
  return_seg.nominal_duration = 1.5;
  return_seg.hand_shape = h1_2_arm_trajectory::HandShape::Open;
  // target_pose将在播放时设置为初始姿势

  auto segments = config.motion_sequence;
  // 注意：这里需要在运行时获取initial pose，暂时留空最后一段
  // 实际实现中应该在player内部处理

  // 播放手势
  auto result = arm_player->play_gesture(segments, config.trajectory_config);

  // 保存结果
  {
    std::lock_guard<std::mutex> lock(gesture_state.mutex);
    gesture_state.result_code = result.error_code;
    gesture_state.result_message = result.message;
    gesture_state.result_ready = true;
    gesture_state.cv.notify_one();
  }

  // 发送响应
  const std::string status = (result.error_code == 0) ? "ok " : "error ";
  const std::string response = status + std::to_string(result.error_code) + " " + result.message;
  send_response(sock, gesture_state.client_addr, gesture_state.client_addr_len, response);

  gesture_state.is_playing.store(false);
}

}  // namespace

int main(int argc, char** argv) {
  std::string network_interface;
  int domain_id = 0;
  int udp_port = 39001;
  double max_x = 0.10;
  double max_y = 0.0;
  double max_yaw = 0.30;
  double shake_hand_duration = 10.0;
  double command_timeout = 1.0;
  bool auto_start = false;
  bool started = false;
  bool shake_hand_extended = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << arg << std::endl;
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--network-interface") network_interface = next();
    else if (arg == "--domain-id") domain_id = std::atoi(next());
    else if (arg == "--udp-port") udp_port = std::atoi(next());
    else if (arg == "--max-x") max_x = std::atof(next());
    else if (arg == "--max-y") max_y = std::atof(next());
    else if (arg == "--max-yaw") max_yaw = std::atof(next());
    else if (arg == "--shake-hand-duration") shake_hand_duration = std::atof(next());
    else if (arg == "--command-timeout") command_timeout = std::atof(next());
    else if (arg == "--grab-pull-config") grab_pull_config_path = next();
    else if (arg == "--auto-start") auto_start = true;
    else if (arg == "--auto-start-value") {
      std::string value = next();
      auto_start = (value == "true" || value == "1" || value == "yes");
    }
  }
  if (!std::isfinite(command_timeout) || command_timeout <= 0.0) {
    std::cerr << "--command-timeout must be a positive finite number" << std::endl;
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

  // 初始化arm_sdk播放器
  arm_player = std::make_unique<h1_2_arm_gesture::ArmGesturePlayer>();
  if (!arm_player->init_dds(domain_id, network_interface)) {
    std::cerr << "Failed to initialize arm gesture player: " << arm_player->last_error() << std::endl;
    return 1;
  }

  unitree::robot::h1::LocoClient client;
  client.Init();
  client.SwitchMoveMode(true);
  if (auto_start) {
    const auto ret = client.Start();
    started = (ret == 0);
    std::cout << "H1-2 loco Start ret=" << ret << std::endl;
  }

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("socket");
    client.StopMove();
    return 1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(udp_port));
  if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    close(sock);
    client.StopMove();
    return 1;
  }

  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  std::cout << "H1-2 loco UDP daemon listening on 127.0.0.1:" << udp_port
            << ", iface=" << network_interface << std::endl;

  char buf[256];
  auto last_control = std::chrono::steady_clock::now();
  bool timed_out = false;

  while (running) {
    if (!timed_out &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_control).count() > command_timeout) {
      const auto ret = client.StopMove();
      started = false;
      timed_out = true;
      std::cout << "Control command timeout; StopMove ret=" << ret << std::endl;
    }

    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                               reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      perror("recvfrom");
      break;
    }
    if (n == 0) {
      continue;
    }
    buf[n] = '\0';

    std::istringstream in{std::string(buf)};
    std::string cmd;
    in >> cmd;

    if (cmd == "start") {
      const auto ret = client.Start();
      started = (ret == 0);
      std::cout << "Start ret=" << ret << std::endl;
      send_response(sock, peer, peer_len,
                    std::string(ret == 0 ? "ok " : "error ") + std::to_string(ret) + " start");

    } else if (cmd == "stop") {
      // 如果正在播放手势，请求中止
      if (gesture_state.is_playing.load()) {
        gesture_state.abort_requested.store(true);
        arm_player->request_abort();
      }

      const auto ret = client.StopMove();
      started = false;
      std::cout << "StopMove ret=" << ret << std::endl;
      send_response(sock, peer, peer_len,
                    std::string(ret == 0 ? "ok " : "error ") + std::to_string(ret) + " stop");

    } else if (cmd == "vel") {
      // 播放手势期间拒绝速度命令
      if (gesture_state.is_playing.load()) {
        send_response(sock, peer, peer_len, "error -3 gesture in progress");
        continue;
      }

      double vx = 0.0, vy = 0.0, wz = 0.0, duration = 0.5;
      bool valid = static_cast<bool>(in >> vx >> vy >> wz);
      if (valid) {
        std::string duration_token;
        if (in >> duration_token) {
          char* end = nullptr;
          duration = std::strtod(duration_token.c_str(), &end);
          valid = end != duration_token.c_str() && *end == '\0';
        }
        std::string extra;
        if (in >> extra) valid = false;
      }
      if (!valid || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
          !std::isfinite(duration) || duration <= 0.0) {
        send_response(sock, peer, peer_len, "error -2 invalid velocity command");
        continue;
      }
      last_control = std::chrono::steady_clock::now();
      timed_out = false;
      vx = clamp(vx, -max_x, max_x);
      vy = clamp(vy, -max_y, max_y);
      wz = clamp(wz, -max_yaw, max_yaw);
      if (!started && (std::abs(vx) > 1e-4 || std::abs(vy) > 1e-4 || std::abs(wz) > 1e-4)) {
        const auto start_ret = client.Start();
        started = (start_ret == 0);
        std::cout << "Auto Start ret=" << start_ret << std::endl;
      }
      const auto ret = client.SetVelocity(static_cast<float>(vx), static_cast<float>(vy),
                                          static_cast<float>(wz), static_cast<float>(duration));
      if (ret != 0) {
        std::cerr << "SetVelocity ret=" << ret << std::endl;
      } else if (std::abs(vx) > 1e-4 || std::abs(vy) > 1e-4 || std::abs(wz) > 1e-4) {
        std::cout << "SetVelocity OK vx=" << vx << " vy=" << vy << " wz=" << wz << std::endl;
      }

    } else if (cmd == "gesture") {
      std::string name;
      in >> name;
      if (name.empty()) {
        send_response(sock, peer, peer_len, "error -2 missing gesture name");
        continue;
      }

      int ret = 0;
      std::string message;

      if (name == "grab_pull") {
        // 检查是否正在播放
        if (gesture_state.is_playing.load()) {
          send_response(sock, peer, peer_len, "error -3 gesture already in progress");
          continue;
        }

        // 检查是否收到状态
        if (!arm_player->wait_for_state(2.0)) {
          send_response(sock, peer, peer_len, "error -6 " + arm_player->last_error());
          continue;
        }

        // 停止运控移动
        client.StopMove();

        // 保存客户端地址
        gesture_state.client_addr = peer;
        gesture_state.client_addr_len = peer_len;
        gesture_state.abort_requested.store(false);
        gesture_state.result_ready = false;
        gesture_state.is_playing.store(true);

        // 启动工作线程
        if (gesture_state.worker_thread.joinable()) {
          gesture_state.worker_thread.join();
        }
        gesture_state.worker_thread = std::thread(gesture_worker_thread, sock);

        std::cout << "Gesture grab_pull started in background" << std::endl;
        // 不立即回复，由工作线程完成后回复
        continue;

      } else if (name == "wave_hand") {
        client.StopMove();
        ret = client.WaveHand();
        message = "wave_hand sent";
      } else if (name == "shake_hand") {
        client.StopMove();
        const int stage = shake_hand_extended ? 1 : 0;
        ret = client.ShakeHand(stage);
        if (ret == 0) shake_hand_extended = !shake_hand_extended;
        message = (ret == 0) ? (shake_hand_extended ? "shake_hand extended" : "shake_hand retracted")
                             : (shake_hand_extended ? "shake_hand retract failed" : "shake_hand extend failed");
      } else if (name == "reach_hand") {
        client.StopMove();
        ret = client.ShakeHand(0);
        message = (ret == 0) ? "reach_hand sent" : "reach_hand failed";
      } else if (name == "retract_hand") {
        client.StopMove();
        ret = client.ShakeHand(1);
        message = (ret == 0) ? "retract_hand sent" : "retract_hand failed";
      } else if (name == "low_stand") {
        client.StopMove();
        ret = client.LowStand();
        message = (ret == 0) ? "low_stand sent" : "low_stand failed";
      } else if (name == "high_stand") {
        client.StopMove();
        ret = client.HighStand();
        message = (ret == 0) ? "high_stand sent" : "high_stand failed";
      } else if (name == "set_task_id") {
        int task_id = -1;
        in >> task_id;
        if (task_id < 0) {
          send_response(sock, peer, peer_len, "error -2 missing task id");
          continue;
        }
        client.StopMove();
        ret = client.SetTaskId(task_id);
        message = "set_task_id " + std::to_string(task_id);
      } else {
        send_response(sock, peer, peer_len, "error -2 unsupported gesture " + name);
        continue;
      }

      const std::string status = (ret == 0) ? "ok " : "error ";
      std::cout << "Gesture " << name << " ret=" << ret << " " << message << std::endl;
      send_response(sock, peer, peer_len, status + std::to_string(ret) + " " + message);

    } else {
      send_response(sock, peer, peer_len, "error -2 unsupported command " + cmd + " " + rest_of_line(in));
    }
  }

  // 清理
  std::cout << "Shutting down..." << std::endl;

  if (gesture_state.is_playing.load()) {
    std::cout << "Aborting gesture playback..." << std::endl;
    gesture_state.abort_requested.store(true);
    arm_player->request_abort();

    if (gesture_state.worker_thread.joinable()) {
      gesture_state.worker_thread.join();
    }
  }

  client.StopMove();
  close(sock);
  return 0;
}
