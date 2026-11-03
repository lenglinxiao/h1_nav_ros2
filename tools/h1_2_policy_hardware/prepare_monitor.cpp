#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>

using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
using Clock = std::chrono::steady_clock;
struct Capture {
  std::mutex mutex;
  LS state{}; LC command{};
  unsigned long states = 0, commands = 0, bad_crc = 0;
  double peak_dq = 0;
  Clock::time_point received{};
  Clock::time_point command_received{};
};
template<class T> bool crc_ok(T value) {
  return value.crc() == crc32_core(reinterpret_cast<uint32_t*>(&value), (sizeof(value) >> 2) - 1);
}
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ofstream trace(argv[2]);
  if (!trace) return 4;
  trace << "time,monotonic,state_received_monotonic,command_received_monotonic,states,commands,peak_dq,command_tau_max,command_native_damping,command_mode_machine,command_mode_pr";
  for(int i=0;i<27;++i) trace << ",q" << i << ",dq" << i << ",tau_est" << i << ",state_mode" << i << ",cmd_tau" << i << ",cmd_mode" << i << ",cmd_kp" << i << ",cmd_kd" << i << ",cmd_q" << i << ",cmd_dq" << i;
  trace << '\n';
  trace.flush();
  if (!trace) return 4;
  auto data = std::make_shared<Capture>();
  unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
  unitree::robot::ChannelSubscriber<LS> states("rt/lowstate");
  unitree::robot::ChannelSubscriber<LC> commands("rt/lowcmd");
  states.InitChannel([data](const void* p) {
    const auto& state = *static_cast<const LS*>(p);
    std::lock_guard<std::mutex> lock(data->mutex);
    if (!crc_ok(state)) { ++data->bad_crc; return; }
    data->state = state; ++data->states; data->received = Clock::now();
    for (int i=0;i<27;++i)
      data->peak_dq = std::max(data->peak_dq, double(std::abs(state.motor_state()[i].dq())));
  }, 1);
  commands.InitChannel([data](const void* p) {
    const auto& command = *static_cast<const LC*>(p);
    std::lock_guard<std::mutex> lock(data->mutex);
    if (!crc_ok(command)) { ++data->bad_crc; return; }
    data->command = command; ++data->commands; data->command_received = Clock::now();
  }, 1);
  const auto start = Clock::now();
  while (Clock::now()-start < std::chrono::seconds(30)) {
    LS s; LC c; unsigned long ns,nc,errors; double peak,age,command_age, state_time, command_time;
    { std::lock_guard<std::mutex> lock(data->mutex);
      s=data->state; c=data->command; ns=data->states; nc=data->commands;
      state_time=std::chrono::duration<double>(data->received.time_since_epoch()).count();
      command_time=std::chrono::duration<double>(data->command_received.time_since_epoch()).count();
      errors=data->bad_crc; peak=data->peak_dq;
      age=ns?std::chrono::duration<double>(Clock::now()-data->received).count():1e6;
      command_age=nc?std::chrono::duration<double>(Clock::now()-data->command_received).count():1e6;
    }
    bool finite=true, damping=nc>0; double tau_max=0;
    for(int i=0;i<27;++i) {
      finite &= std::isfinite(s.motor_state()[i].q()) && std::isfinite(s.motor_state()[i].dq());
      const auto& m=c.motor_cmd()[i];
      finite &= std::isfinite(m.tau()) && std::isfinite(m.kp()) && std::isfinite(m.kd()) && std::isfinite(m.q()) && std::isfinite(m.dq()) && std::isfinite(s.motor_state()[i].tau_est());
      tau_max=std::max(tau_max,double(std::abs(m.tau())));
      damping &= m.tau()==0 && m.kp()==0 && m.kd()>0 && m.dq()==0;
    }
    double t=std::chrono::duration<double>(Clock::now()-start).count();
    const std::string temporary=std::string(argv[1])+".tmp";
    { std::ofstream out(temporary); out << std::setprecision(10)
        << "{\"schema\":2,\"time\":"<<t<<",\"states\":"<<ns<<",\"commands\":"<<nc
        <<",\"bad_crc\":"<<errors<<",\"peak_dq\":"<<peak<<",\"age\":"<<age
        <<",\"command_age\":"<<command_age
        <<",\"finite\":"<<(finite?"true":"false")
        <<",\"machine\":"<<unsigned(s.mode_machine())<<",\"pr\":"<<unsigned(s.mode_pr())
        <<",\"native_damping\":"<<(damping?"true":"false")
        <<",\"tau_max\":"<<tau_max<<",\"q\":[";
      for(int i=0;i<27;++i) out<<(i?",":"")<<s.motor_state()[i].q();
      out<<"],\"dq\":[";
      for(int i=0;i<27;++i) out<<(i?",":"")<<s.motor_state()[i].dq();
      out<<"]}\n";
      if(!out) return 3;
    }
    if(std::rename(temporary.c_str(),argv[1])!=0) return 3;
    trace<<std::setprecision(12)<<t<<','<<std::chrono::duration<double>(Clock::now().time_since_epoch()).count()<<','<<state_time<<','<<command_time<<','<<ns<<','<<nc<<','<<peak<<','<<tau_max<<','<<damping<<','<<unsigned(c.mode_machine())<<','<<unsigned(c.mode_pr());
    for(int i=0;i<27;++i) trace<<','<<s.motor_state()[i].q()<<','<<s.motor_state()[i].dq()<<','<<s.motor_state()[i].tau_est()<<','<<unsigned(s.motor_state()[i].mode())<<','<<c.motor_cmd()[i].tau()<<','<<unsigned(c.motor_cmd()[i].mode())<<','<<c.motor_cmd()[i].kp()<<','<<c.motor_cmd()[i].kd()<<','<<c.motor_cmd()[i].q()<<','<<c.motor_cmd()[i].dq();
    trace<<'\n';trace.flush();
    if (!trace) return 4;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  states.CloseChannel(); commands.CloseChannel();
}
