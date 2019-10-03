
#ifndef TELLO_INTERFACE_HPP
#define TELLO_INTERFACE_HPP

#include "tello_msgs/msg/tello_response.hpp"
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

    virtual Result on_tello_response(tello_msgs::msg::TelloResponse::SharedPtr &msg);
  };
}


#endif //TELLO_INTERFACE_HPP
