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

}  // namespace

int main(int argc, char** argv) {
  std::string network_interface;
  int domain_id = 0;
  int udp_port = 39001;
  double max_x = 0.10;
  double max_y = 0.0;
  double max_yaw = 0.30;
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
    std::cout << "H1/H1-2 loco Start ret=" << ret << std::endl;
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

  std::cout << "H1/H1-2 loco UDP daemon listening on 127.0.0.1:" << udp_port
            << ", iface=" << network_interface << std::endl;

  char buf[256];
  while (running) {
    const ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
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
    }
  }

  client.StopMove();
  close(sock);
  return 0;
}
