
#include "sm_pause.hpp"

namespace provoke
{
  namespace sm_pause
  {
    void Hub::set_ready(rclcpp::Duration duration)
    {
      machine_.ready_.prepare(duration);
      machine_.set_state(machine_.ready_);
    }

    void Hub::set_waiting(rclcpp::Time end_time)
    {
      machine_.waiting_.prepare(end_time);
      machine_.set_state(machine_.waiting_);
    }
  }

  std::unique_ptr<sm_pause::Machine> sm_pause_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_pause::Machine>(impl);
  }

  void sm_prepare(sm_pause::Machine &machine, rclcpp::Duration duration)
  {
    machine.prepare(duration);
  }

}