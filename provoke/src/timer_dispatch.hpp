
#ifndef TIMER_DISPATCH_HPP
#define TIMER_DISPATCH_HPP

#include "timer_interface.hpp"

namespace provoke
{
  class TimerDispatch : public TimerInterface
  {
    std::unique_ptr<TimerInterface> timer_machine_pause_;

  public:
    TimerDispatch(ProvokeNodeImpl &impl);

    ~TimerDispatch() override;

    Result on_timer(rclcpp::Time now) override;
  };
}
#endif //TIMER_DISPATCH_HPP
