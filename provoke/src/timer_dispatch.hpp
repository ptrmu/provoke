
#ifndef TIMER_DISPATCH_HPP
#define TIMER_DISPATCH_HPP

#include "timer_interface.hpp"

namespace provoke
{
  class TimerDispatch : public TimerInterface
  {
    std::unique_ptr<TimerInterface> timer_machine_pause_;

  public:
    int group_index_;

    TimerDispatch(ProvokeNodeImpl &impl, int group_index);

    ~TimerDispatch() override;

    Result on_timer(rclcpp::Time now) override;

    Result validate_args(YamlArgs &args) override;

    Result prepare_from_args(YamlArgs &args) override;
  };

  namespace timer_machine_pause
  {
    std::unique_ptr<TimerInterface> factory(TimerDispatch &dispatch);
  }

  namespace timer_machine_par
  {
    std::unique_ptr<TimerInterface> factory(TimerDispatch &dispatch);
  }
}
#endif //TIMER_DISPATCH_HPP
