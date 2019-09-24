#ifndef PROVOKE_NODE_IMPL_HPP
#define PROVOKE_NODE_IMPL_HPP

#include <memory>

#include "state_machine_interface.hpp"


namespace provoke
{
  class TimerDispatch;

  class TelloDispatch;

  class TimerInterface;

  class ProvokeNodeImpl
  {
  public:
    rclcpp::Node &node_;
    const long timer_interval_ms = 10;
    std::unique_ptr<TimerDispatch> timer_dispatch_;
    std::vector<std::unique_ptr<TimerDispatch>> timer_dispatches_;
    std::vector<std::unique_ptr<TelloDispatch>> tello_dispatches_;

  private:
    std::unique_ptr<sm_manager::Machine> sm_manager_;
    std::unique_ptr<TimerInterface> base_machine_;
    rclcpp::TimerBase::SharedPtr timer_{};

  public:
    ProvokeNodeImpl(rclcpp::Node &node);

    ~ProvokeNodeImpl();
  };
}

#endif //PROVOKE_NODE_IMPL_HPP
