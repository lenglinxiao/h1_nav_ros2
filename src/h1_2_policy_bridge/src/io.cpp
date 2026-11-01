#include "h1_2_policy_bridge/io.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>
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
    pub_ = std::make_shared<unitree::robot::ChannelPublisher<LC>>("rt/lowcmd");
    pub_->InitChannel();
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
  std::shared_ptr<unitree::robot::ChannelPublisher<LC>> pub_;
};
std::unique_ptr<LowLevelIo> make_sdk_io() { return std::make_unique<SdkIo>(); }
}
