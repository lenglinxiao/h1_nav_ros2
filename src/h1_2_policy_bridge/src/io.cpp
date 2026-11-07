#include "h1_2_policy_bridge/io.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace h1_policy {
using LS = unitree_hg::msg::dds_::LowState_;
using LC = unitree_hg::msg::dds_::LowCmd_;
class SdkIo final : public LowLevelIo {
  struct CallbackState { std::mutex mutex; StateCallback callback; };
public:
  ~SdkIo() override { stop(); }
  void start(int domain, const std::string& interface, StateCallback callback) override {
    callback_state_ = std::make_shared<CallbackState>();
    callback_state_->callback = std::move(callback);
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LS>>("rt/lowstate");
    auto state = callback_state_;
    sub_->InitChannel([state](const void* p) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->callback) state->callback(*static_cast<const LS*>(p));
    });
  }
  void open_output() override {
    auto pub = unitree::robot::ChannelFactory::Instance()->CreateSendChannel<LC>("rt/lowcmd");
    const char* path = std::getenv("H1_POLICY_WRITER_GUID_FILE");
    if (path && *path) {
      dds_guid_t guid{};
      auto writer = pub->GetWriter()->GetNative();
      if (dds_get_guid(writer.delegate()->get_ddsc_entity(), &guid) != DDS_RETCODE_OK)
        throw std::runtime_error("cannot obtain LowCmd writer GUID");
      const std::string temporary = std::string(path) + ".tmp";
      { std::ofstream out(temporary, std::ios::trunc);
        for (auto byte : guid.v) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
        out << '\n'; out.close();
        if (!out) throw std::runtime_error("cannot write LowCmd writer GUID");
      }
      if (std::rename(temporary.c_str(), path) != 0)
        throw std::runtime_error("cannot publish LowCmd writer GUID");
    }
    pub_ = std::move(pub);
  }
  bool write(const LC& message) override { return pub_ && pub_->Write(message, 0); }
  void close_output() override { pub_.reset(); }
  bool output_present() const override { return bool(pub_); }
  void stop() override {
    if (callback_state_) {
      std::lock_guard<std::mutex> lock(callback_state_->mutex);
      callback_state_->callback = {};
    }
    sub_.reset(); close_output(); callback_state_.reset();
  }
private:
  std::shared_ptr<CallbackState> callback_state_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<LS>> sub_;
  unitree::robot::ChannelPtr<LC> pub_;
};
std::unique_ptr<LowLevelIo> make_sdk_io() { return std::make_unique<SdkIo>(); }
}
