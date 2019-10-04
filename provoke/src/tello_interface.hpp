
#ifndef TELLO_INTERFACE_HPP
#define TELLO_INTERFACE_HPP

#include "tello_msgs/msg/tello_response.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include "args_interface.hpp"

namespace provoke
{
  class TelloInterface : public ArgsInterface
  {
  public:
    TelloInterface(std::string name, ProvokeNodeImpl &impl) :
      ArgsInterface{std::move(name), impl}
    {}

    TelloInterface() = delete;

    ~TelloInterface() override = default;

    virtual Result on_tello_response(const tello_msgs::msg::TelloResponse &response);

    virtual Result on_tello_action_response(const tello_msgs::srv::TelloAction_Response &action_response);
  };
}

#endif //TELLO_INTERFACE_HPP
