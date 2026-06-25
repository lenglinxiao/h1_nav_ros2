#include <arpa/inet.h>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/h1/loco/h1_loco_client.hpp"

namespace {
volatile std::sig_atomic_t running = 1;

void handle_signal(int) {
  running = 0;
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

}  // namespace

int main(int argc, char** argv) {
  std::string network_interface;
  int domain_id = 0;
  int udp_port = 39001;
  double max_x = 0.10;
  double max_y = 0.0;
  double max_yaw = 0.30;
  double shake_hand_duration = 10.0;
  bool auto_start = false;
  bool started = false;

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
    else if (arg == "--auto-start") auto_start = true;
    else if (arg == "--auto-start-value") {
      std::string value = next();
      auto_start = (value == "true" || value == "1" || value == "yes");
    }
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
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
    return 1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(udp_port));
  if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    close(sock);
    return 1;
  }

  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  std::cout << "H1-2 loco UDP daemon listening on 127.0.0.1:" << udp_port
            << ", iface=" << network_interface << std::endl;

  char buf[256];
  while (running) {
    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                               reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (n <= 0) {
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
    } else if (cmd == "vel") {
      double vx = 0.0, vy = 0.0, wz = 0.0, duration = 0.5;
      in >> vx >> vy >> wz >> duration;
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
      if (name == "wave_hand") {
        client.StopMove();
        ret = client.WaveHand();
        message = "wave_hand sent";
      } else if (name == "shake_hand") {
        double duration = shake_hand_duration;
        in >> duration;
        if (duration < 0.0) {
          duration = shake_hand_duration;
        }
        client.StopMove();
        const int start_ret = client.ShakeHand(0);
        if (start_ret != 0) {
          ret = start_ret;
          message = "shake_hand start failed";
        } else {
          const auto sleep_time = std::chrono::milliseconds(static_cast<int>(duration * 1000.0));
          std::this_thread::sleep_for(sleep_time);
          ret = client.ShakeHand(1);
          message = (ret == 0) ? "shake_hand finished" : "shake_hand finish failed";
        }
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

  client.StopMove();
  close(sock);
  return 0;
}
