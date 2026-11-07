#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <algorithm>
#include <cctype>

using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
struct Capture {
  std::mutex mutex;
  LS state{}; LC command{};
  unsigned long states = 0, commands = 0, bad_crc = 0, state_bad_crc = 0;
  double peak_dq = 0;
  std::array<double, 27> peak_dq_by_joint{};
  Clock::time_point received{};
  Clock::time_point command_received{};
};
template<class T> bool crc_ok(T value) {
  return value.crc() == crc32_core(reinterpret_cast<uint32_t*>(&value), (sizeof(value) >> 2) - 1);
}
int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) return 2;
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::ofstream trace(argv[2]);
  if (!trace) return 4;
  trace << "time,monotonic,state_received_monotonic,command_received_monotonic,states,commands,peak_dq,command_tau_max,command_native_damping,command_mode_machine,command_mode_pr,policy_writer_guid,policy_commands,policy_command_age,policy_bad_crc,policy_finite,policy_native_damping,policy_tau_max,unknown_commands";
  for(int i=0;i<27;++i) trace << ",q" << i << ",dq" << i << ",tau_est" << i << ",state_mode" << i << ",cmd_tau" << i << ",cmd_mode" << i << ",cmd_kp" << i << ",cmd_kd" << i << ",cmd_q" << i << ",cmd_dq" << i;
  trace << '\n';
  trace.flush();
  if (!trace) return 4;
  auto data = std::make_shared<Capture>();
  unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
  unitree::robot::ChannelSubscriber<LS> states("rt/lowstate");
  ::dds::domain::DomainParticipant participant(0);
  ::dds::sub::Subscriber subscriber(participant);
  ::dds::topic::Topic<LC> topic(participant, "rt/lowcmd");
  auto qos = subscriber.default_datareader_qos();
  qos << ::dds::core::policy::History::KeepLast(128);
  ::dds::sub::DataReader<LC> reader(subscriber, topic, qos);
  LC policy_command{};
  unsigned long policy_commands=0, policy_bad_crc=0, unknown_commands=0;
  Clock::time_point policy_received{};
  std::string expected_guid;
  states.InitChannel([data](const void* p) {
    const auto& state = *static_cast<const LS*>(p);
    std::lock_guard<std::mutex> lock(data->mutex);
    if (!crc_ok(state)) { ++data->bad_crc; ++data->state_bad_crc; return; }
    data->state = state; ++data->states; data->received = Clock::now();
    for (int i=0;i<27;++i) {
      const double dq = double(std::abs(state.motor_state()[i].dq()));
      data->peak_dq = std::max(data->peak_dq, dq);
      data->peak_dq_by_joint[i] = std::max(data->peak_dq_by_joint[i], dq);
    }
  }, 1);
  const auto start = Clock::now();
  while (!stop_requested) {
    { std::ifstream file(argc == 4 ? argv[3] : ""); std::string guid; file >> guid;
      if (!guid.empty() && guid.size()==32 && std::all_of(guid.begin(),guid.end(),[](unsigned char c){return (c>='0' && c<='9') || (c>='a' && c<='f');})) {
        if (expected_guid.empty()) expected_guid=guid;
        else if (expected_guid!=guid) return 5;
      } else if (!expected_guid.empty()) return 5;
    }
    auto samples = reader.take();
    for (const auto& sample : samples) {
      if (!sample.info().valid()) continue;
      auto* endpoint=dds_get_matched_publication_data(reader.delegate()->get_ddsc_entity(), sample.info().publication_handle().delegate().handle());
      std::string guid;
      if (endpoint) {
        std::ostringstream hex;
        for (auto byte : endpoint->key.v) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
        guid=hex.str(); dds_builtintopic_free_endpoint(endpoint);
      }
      bool own=!expected_guid.empty() && guid==expected_guid;
      const auto& command=sample.data();
      if (!own) ++unknown_commands;
      if (!crc_ok(command)) { std::lock_guard<std::mutex> lock(data->mutex); ++data->bad_crc; if (own) ++policy_bad_crc; continue; }
      { std::lock_guard<std::mutex> lock(data->mutex);
        data->command=command; ++data->commands; data->command_received=Clock::now();
      }
      if (own) { policy_command=command; ++policy_commands; policy_received=Clock::now(); }
    }
    LS s; LC c; unsigned long ns,nc,errors,state_errors; double peak,age,command_age, state_time, command_time;
    std::array<double, 27> joint_peaks;
    { std::lock_guard<std::mutex> lock(data->mutex);
      s=data->state; c=data->command; ns=data->states; nc=data->commands;
      state_time=std::chrono::duration<double>(data->received.time_since_epoch()).count();
      command_time=std::chrono::duration<double>(data->command_received.time_since_epoch()).count();
      errors=data->bad_crc; state_errors=data->state_bad_crc; peak=data->peak_dq; joint_peaks=data->peak_dq_by_joint;
      age=ns?std::chrono::duration<double>(Clock::now()-data->received).count():1e6;
      command_age=nc?std::chrono::duration<double>(Clock::now()-data->command_received).count():1e6;
    }
    bool finite=true, state_finite=ns>0, damping=nc>0; double tau_max=0;
    for(int i=0;i<27;++i) {
      state_finite &= std::isfinite(s.motor_state()[i].q()) && std::isfinite(s.motor_state()[i].dq()) && std::isfinite(s.motor_state()[i].tau_est());
      finite &= state_finite;
      const auto& m=c.motor_cmd()[i];
      finite &= std::isfinite(m.tau()) && std::isfinite(m.kp()) && std::isfinite(m.kd()) && std::isfinite(m.q()) && std::isfinite(m.dq()) && std::isfinite(s.motor_state()[i].tau_est());
      tau_max=std::max(tau_max,double(std::abs(m.tau())));
      damping &= m.tau()==0 && m.kp()==0 && m.kd()>0 && m.dq()==0;
    }
    bool policy_finite=policy_commands>0, policy_damping=policy_commands>0; double policy_tau_max=0;
    for (int i=0;i<27;++i) {
      const auto& m=policy_command.motor_cmd()[i];
      policy_finite &= std::isfinite(m.tau()) && std::isfinite(m.kp()) && std::isfinite(m.kd()) && std::isfinite(m.q()) && std::isfinite(m.dq());
      policy_damping &= m.tau()==0 && m.kp()==0 && m.kd()>0 && m.dq()==0;
      policy_tau_max=std::max(policy_tau_max,double(std::abs(m.tau())));
    }
    double policy_age=policy_commands?std::chrono::duration<double>(Clock::now()-policy_received).count():1e6;
    double t=std::chrono::duration<double>(Clock::now()-start).count();
    const std::string temporary=std::string(argv[1])+".tmp";
    { std::ofstream out(temporary); out << std::setprecision(10)
        << "{\"schema\":3,\"lifetime\":\"until_stopped\",\"time\":"<<t<<",\"states\":"<<ns<<",\"commands\":"<<nc
        <<",\"state_bad_crc\":"<<state_errors
        <<",\"state_finite\":"<<(state_finite?"true":"false")
        <<",\"policy_writer_guid\":\""<<expected_guid<<"\""
        <<",\"policy_commands\":"<<policy_commands<<",\"policy_bad_crc\":"<<policy_bad_crc
        <<",\"policy_command_age\":"<<policy_age<<",\"policy_finite\":"<<(policy_finite?"true":"false")
        <<",\"policy_native_damping\":"<<(policy_damping?"true":"false")<<",\"policy_tau_max\":"<<policy_tau_max
        <<",\"unknown_commands\":"<<unknown_commands
        <<",\"bad_crc\":"<<errors<<",\"peak_dq\":"<<peak<<",\"age\":"<<age
        <<",\"peak_dq_by_joint\":[";
      for(int i=0;i<27;++i) out<<(i?",":"")<<joint_peaks[i];
      out <<"],\"command_age\":"<<command_age
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
    trace<<std::setprecision(12)<<t<<','<<std::chrono::duration<double>(Clock::now().time_since_epoch()).count()<<','<<state_time<<','<<command_time<<','<<ns<<','<<nc<<','<<peak<<','<<tau_max<<','<<damping<<','<<unsigned(c.mode_machine())<<','<<unsigned(c.mode_pr())<<','<<expected_guid<<','<<policy_commands<<','<<policy_age<<','<<policy_bad_crc<<','<<policy_finite<<','<<policy_damping<<','<<policy_tau_max<<','<<unknown_commands;
    for(int i=0;i<27;++i) trace<<','<<s.motor_state()[i].q()<<','<<s.motor_state()[i].dq()<<','<<s.motor_state()[i].tau_est()<<','<<unsigned(s.motor_state()[i].mode())<<','<<c.motor_cmd()[i].tau()<<','<<unsigned(c.motor_cmd()[i].mode())<<','<<c.motor_cmd()[i].kp()<<','<<c.motor_cmd()[i].kd()<<','<<c.motor_cmd()[i].q()<<','<<c.motor_cmd()[i].dq();
    trace<<'\n';trace.flush();
    if (!trace) return 4;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  states.CloseChannel();
}
