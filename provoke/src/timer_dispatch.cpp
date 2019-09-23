
#include "timer_dispatch.hpp"

#include "provoke_node_impl.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  TimerDispatch::TimerDispatch(ProvokeNodeImpl &impl, int group_index) :
    TimerInterface("TimerDispatch", impl),
    timer_machine_pause_{timer_machine_pause::factory(*this)},
    group_index_{group_index}
  {
  }

  TimerDispatch::~TimerDispatch() = default;

  Result TimerDispatch::on_timer(rclcpp::Time now)
  {
    return timer_machine_pause_->on_timer(now);
  }


  Result TimerDispatch::validate_args(YamlArgs &args)
  {
    return validate_cmd_seq(args, [this](const std::string &cmd) -> std::unique_ptr<TimerInterface>{
      if (cmd.compare("pause") == 0) {
        return timer_machine_pause::factory(*this);
      }
      else if (cmd.compare("par") == 0) {
        return timer_machine_par::factory(*this);
      }
      return std::unique_ptr<TimerInterface>{};
    });
  }

  Result TimerDispatch::prepare_from_args(YamlArgs &args)
  {
    (void)args;
    return Result::failure();
  }
}
