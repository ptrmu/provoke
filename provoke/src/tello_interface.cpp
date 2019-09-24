
#include "tello_interface.hpp"

namespace provoke
{
  Result TelloInterface::on_tello_response(tello_msgs::msg::TelloResponse *msg)
  {
    auto msg_str = msg->str;
    (void) msg;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'on_tello_response()'",
                               name_.c_str());
  }
}