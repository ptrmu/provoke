
#include "timer_dispatch.hpp"

#include "provoke_node_impl.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  TimerDispatch::TimerDispatch(ProvokeNodeImpl &impl, int inst_index, TelloDispatch &tello_dispatch) :
    SharedDispatch("TimerDispatch", impl, inst_index),
    tello_dispatch_{tello_dispatch}
  {
    new_machine_ = [this](const std::string &cmd) -> std::unique_ptr<ArgsInterface>
    {
      if (cmd.compare("pause") == 0) {
        return timer_machine_pause::factory(*this);
      } else if (cmd.compare("par") == 0) {
        return timer_machine_par::factory(*this);
      } else if (cmd.compare("tello") == 0) {
        return timer_machine_tello::factory(*this);
      }
      return std::unique_ptr<ArgsInterface>{};
    };
  }

  TimerDispatch::~TimerDispatch() = default;
}
