
#ifndef TIMER_DISPATCH_HPP
#define TIMER_DISPATCH_HPP

#include "timer_interface.hpp"

namespace provoke
{
  class YamlSeq;

  class TimerDispatch : public TimerInterface
  {
    enum class States
    {
      concluded = 0,
      running,
    };

    States state_{States::concluded};

    std::unique_ptr<YamlSeq> running_seq_;
    std::unique_ptr<TimerInterface> running_machine_;

    std::unique_ptr<TimerInterface> new_machine(const std::string &cmd);

    Result prepare_cmd_from_seq(void);

    Result on_timer_concluded(rclcpp::Time now);

    Result on_timer_running(rclcpp::Time now);

  public:
    int group_index_;

    TimerDispatch(ProvokeNodeImpl &impl, int group_index);

    ~TimerDispatch() override;

    Result on_timer(rclcpp::Time now) override;

    Result validate_args(YamlArgs &args) override;

    Result prepare_from_args(YamlArgs &args) override;

    void set_concluded(void);
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
