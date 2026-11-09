#include "h1_2_policy_bridge/io.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>
#include <algorithm>
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
    dds_guid_t guid{};
    auto writer = pub->GetWriter()->GetNative();
    const bool have_guid = dds_get_guid(writer.delegate()->get_ddsc_entity(), &guid) == DDS_RETCODE_OK;
    const char* path = std::getenv("H1_POLICY_WRITER_GUID_FILE");
    if (path && *path) {
      if (!have_guid)
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
    std::lock_guard<std::mutex> lock(output_mutex_);
    pub_ = std::move(pub);
    guid_present_ = have_guid;
    if (guid_present_) std::copy(std::begin(guid.v), std::end(guid.v), guid_.begin());
  }
  bool write(const LC& message) override { std::lock_guard<std::mutex> lock(output_mutex_); return pub_ && pub_->Write(message, 0); }
  void close_output() override { std::lock_guard<std::mutex> lock(output_mutex_); pub_.reset(); guid_present_ = false; guid_.fill(0); }
  bool output_present() const override { std::lock_guard<std::mutex> lock(output_mutex_); return bool(pub_); }
  bool output_guid(std::array<uint8_t, 16>& guid) const override {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (!pub_ || !guid_present_) return false;
    guid = guid_;
    return true;
  }
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
  mutable std::mutex output_mutex_;
  std::array<uint8_t, 16> guid_{};
  bool guid_present_ = false;
};
std::unique_ptr<LowLevelIo> make_sdk_io() { return std::make_unique<SdkIo>(); }
}
