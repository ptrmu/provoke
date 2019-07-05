
#include "sm_out_back.hpp"

namespace provoke
{
  namespace sm_out_back
  {
    Hub::Hub(Machine &machine) :
      machine_{machine}, gos_{
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_)}
    {}

    void Hub::set_running()
    {
      machine_.running_.prepare();
      machine_.set_state(machine_.running_);
    }

    void Hub::set_complete()
    {
      machine_.set_state(machine_.complete_);
    }
  }

  std::unique_ptr<sm_out_back::Machine> sm_out_back_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_out_back::Machine>(impl);
  }

  void sm_prepare(sm_out_back::Machine &machine, tf2::Vector3 velocity_mps,
                  rclcpp::Duration go_duration, rclcpp::Duration stop_duration, double msg_rate_hz)
  {
    machine.hub_.prepare(velocity_mps, go_duration, stop_duration, msg_rate_hz);
  }
}