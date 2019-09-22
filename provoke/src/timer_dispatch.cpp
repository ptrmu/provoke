
#include "timer_dispatch.hpp"

#include "provoke_node_impl.hpp"

namespace provoke
{
  TimerDispatch::TimerDispatch(ProvokeNodeImpl &impl) :
    TimerInterface("TimerDispatch", impl),
    timer_machine_pause_{timer_machine_pause::factory(*this)}
  {}

  TimerDispatch::~TimerDispatch() = default;

  Result TimerDispatch::on_timer(rclcpp::Time now)
  {
    return timer_machine_pause_->on_timer(now);
  }
}