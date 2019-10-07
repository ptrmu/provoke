#ifndef PROVOKE_NODE_IMPL_HPP
#define PROVOKE_NODE_IMPL_HPP

#include <memory>

#include "state_machine_interface.hpp"


namespace rclcpp
{
  class Node;

  class TimerBase;
}

namespace provoke
{
  class TimerDispatch;

  class TelloDispatch;

  class TimerInterface;

  class ProvokeNodeImpl
  {
  public:
    rclcpp::Node &node_;
    bool inited_{false};
    int init_try_count_{0};
    const long timer_interval_ms = 10;
    std::unique_ptr<TimerDispatch> timer_dispatch_;
    std::vector<std::unique_ptr<TimerDispatch>> timer_dispatches_;
    std::vector<std::unique_ptr<TelloDispatch>> tello_dispatches_;

  private:
    std::unique_ptr<TimerInterface> base_machine_;
    std::shared_ptr<rclcpp::TimerBase> timer_{};

  public:
    explicit ProvokeNodeImpl(rclcpp::Node &node);

    ~ProvokeNodeImpl();
  };
}

#endif //PROVOKE_NODE_IMPL_HPP
