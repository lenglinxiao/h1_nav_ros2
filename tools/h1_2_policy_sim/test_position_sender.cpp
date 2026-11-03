// Loopback-only DDS stimulus for the MuJoCo position-PD regression.
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <chrono>
#include <limits>
#include <thread>
#include <cstdlib>
using LC = unitree_hg::msg::dds_::LowCmd_;
int main(int argc, char** argv) {
 if (argc != 2) return 2;
 const float kd = std::atof(argv[1]);
 unitree::robot::ChannelFactory::Instance()->Init(42, "lo");
 unitree::robot::ChannelPublisher<LC> pub("rt/lowcmd"); pub.InitChannel();
 LC msg{};
 for(int i=0;i<27;i++) { auto& m=msg.motor_cmd()[i]; m.mode()=1; m.kd()=kd; }
 msg.motor_cmd()[3].q()=.7f; msg.motor_cmd()[3].kp()=80;
 msg.motor_cmd()[9].q()=.7f; msg.motor_cmd()[9].kp()=80;
 auto send=[&] { msg.crc()=crc32_core(reinterpret_cast<uint32_t*>(&msg),(sizeof(msg)>>2)-1); if(!pub.Write(msg,0)) std::abort(); };
 std::this_thread::sleep_for(std::chrono::milliseconds(300));
 for(int i=0;i<100;i++) { send(); std::this_thread::sleep_for(std::chrono::milliseconds(20)); }
 msg.motor_cmd()[3].mode()=0; for(int i=0;i<5;i++){send(); std::this_thread::sleep_for(std::chrono::milliseconds(20));}
 std::this_thread::sleep_for(std::chrono::milliseconds(200));
 msg.motor_cmd()[3].mode()=1; msg.motor_cmd()[3].q()=std::numeric_limits<float>::quiet_NaN();
 for(int i=0;i<5;i++){send(); std::this_thread::sleep_for(std::chrono::milliseconds(20));}
}
