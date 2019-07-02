
#include "state_machine_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "provoke_node_impl.hpp"

namespace provoke
{
  StateInterface::StateInterface(ProvokeNodeImpl &impl, std::string name) :
    impl_(impl), name_(name)
  {}

  void StateInterface::on_enter()
  {
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_enter()' not overridden for state %s", name_.c_str());
  }

  bool StateInterface::on_timer()
  {
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_timer()' not overridden for state %s", name_.c_str());
    return false;
  }

}
