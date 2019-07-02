#ifndef PROVOKE_NODE_IMPL_HPP
#define PROVOKE_NODE_IMPL_HPP

#include <memory>

#include "state_machine_interface.hpp"

namespace rclcpp
{
  class Node;
}

namespace sm_manager
{
  class Machine;
}

namespace provoke
{
  class ProvokeNodeImpl
  {
  public:
    rclcpp::Node &node_;

  private:
    std::unique_ptr<StateMachineInterface> sm_manager_;
    std::unique_ptr<StateMachineInterface> sm_send_action_;

  public:
    ProvokeNodeImpl(rclcpp::Node &node);

    ~ProvokeNodeImpl();
  };
}

#endif //PROVOKE_NODE_IMPL_HPP
