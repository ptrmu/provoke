
#include "state_machine_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "provoke_node_impl.hpp"

namespace provoke
{
  void StateInterface::on_enter()
  {
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_enter()' not overridden for state %s", name_.c_str());
  }

  bool StateInterface::on_timer(rclcpp::Time now)
  {
    (void)now;
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_timer()' not overridden for state %s", name_.c_str());
    return false;
  }

  bool StateInterface::on_tello_response(tello_msgs::msg::TelloResponse * msg)
  {
    (void)msg;
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_tello_response()' not overridden for state %s", name_.c_str());
    return false;
  }

  void StateMachineInterface::on_enter()
  {
    RCLCPP_ERROR(impl_.node_.get_logger(), "Method 'on_enter()' not overridden for state machine %s", name_.c_str());
  }
}
