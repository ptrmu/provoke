
#include "tello_interface.hpp"

namespace provoke
{
  Result TelloInterface::on_tello_response(const tello_msgs::msg::TelloResponse &response)
  {
    (void) response;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'on_tello_response()'",
                               name_.c_str());
  }

  Result TelloInterface::on_tello_action_response(const tello_msgs::srv::TelloAction_Response &action_response)
  {
    (void) action_response;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'on_tello_action_response()'",
                               name_.c_str());
  }
}