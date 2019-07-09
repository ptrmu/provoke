
#include "state_machine_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "provoke_node_impl.hpp"

namespace provoke
{
  SMResult StateInterface::on_timer(rclcpp::Time now)
  {
    (void) now;
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_timer()' not overridden for state %s", name_.c_str());
    return SMResult{SMResultCodes::failure, "on_timer un-implemented"};
  }

  SMResult StateInterface::on_tello_response(tello_msgs::msg::TelloResponse *msg)
  {
    (void) msg;
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_tello_response()' not overridden for state %s", name_.c_str());
    return SMResult{SMResultCodes::failure, "on_tello_response un-implemented"};
  }

  SMResult StateMachineInterface::validate_args(const StateMachineArgs &args)
  {
    (void) args;
    return SMResult{SMResultCodes::failure, "validate_args un-implemented"};
  }

  SMResult StateMachineInterface::prepare_from_args(const StateMachineArgs &args)
  {
    (void) args;
    return SMResult{SMResultCodes::failure, "prepare_from_args un-implemented"};
  }
}
