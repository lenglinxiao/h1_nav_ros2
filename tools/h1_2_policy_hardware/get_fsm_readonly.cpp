// Read-only H1 loco GET_FSM_ID diagnostic. No LowCmd or mode-changing API.
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

class ReadOnlyLoco final : public unitree::robot::h1::LocoClient {
 public:
  int32_t get_fsm_raw(std::string& response) {
    const std::string parameter;
    return Call(unitree::robot::h1::ROBOT_API_ID_LOCO_GET_FSM_ID, parameter, response);
  }
};

int main() {
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    ReadOnlyLoco client;
    client.SetTimeout(3.0f);
    client.Init();
    std::string response;
    const int32_t ret = client.get_fsm_raw(response);
    std::cout << "get_fsm_ret=" << ret << " raw_response=" << std::quoted(response) << std::endl;
    if (ret != 0) return 1;
    try {
      unitree::robot::go2::JsonizeDataInt parsed;
      unitree::common::FromJsonString(response, parsed);
      std::cout << "fsm_id=" << parsed.data << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "response_parse_error=" << e.what() << std::endl;
      return 2;
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "query_exception=" << e.what() << std::endl;
    return 3;
  }
}
