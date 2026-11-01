#pragma once
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <functional>
#include <memory>
#include <string>
namespace h1_policy {
class LowLevelIo {
public:
  using StateCallback = std::function<void(const unitree_hg::msg::dds_::LowState_&)>;
  virtual ~LowLevelIo() = default;
  virtual void start(int domain, const std::string& interface, StateCallback callback) = 0;
  virtual void open_output() = 0;
  virtual bool write(const unitree_hg::msg::dds_::LowCmd_&) = 0;
  virtual void close_output() = 0;
  virtual bool output_present() const = 0;
  // After stop returns no registered callback may execute or start.
  virtual void stop() = 0;
};
std::unique_ptr<LowLevelIo> make_sdk_io();
}
