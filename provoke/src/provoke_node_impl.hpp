#ifndef PROVOKE_NODE_IMPL_HPP
#define PROVOKE_NODE_IMPL_HPP

#include <memory>

#include "state_machine_interface.hpp"

namespace provoke
{
  class ProvokeNodeImpl
  {
  public:
    rclcpp::Node &node_;
    const long timer_interval_ms = 10;

  private:
    std::unique_ptr<sm_manager::Machine> sm_manager_;
    rclcpp::TimerBase::SharedPtr timer_{};


  public:
    ProvokeNodeImpl(rclcpp::Node &node);

    ~ProvokeNodeImpl();
  };
}

#endif //PROVOKE_NODE_IMPL_HPP
