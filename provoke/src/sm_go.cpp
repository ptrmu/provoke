
#include "sm_go.hpp"

namespace provoke
{
  namespace sm_go
  {
    void Hub::set_ready()
    {
      machine_.set_state(machine_.ready_);
    }

    void Hub::set_waiting(rclcpp::Time end_time, rclcpp::Time next_msg_time)
    {
      machine_.waiting_.prepare(end_time, next_msg_time);
      machine_.set_state(machine_.waiting_);
    }
  }

  std::unique_ptr<sm_go::Machine> sm_go_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_go::Machine>(impl);
  }

  void sm_prepare(sm_go::Machine &machine, tf2::Vector3 velocity_mps,
                  rclcpp::Duration duration, double msg_rate_hz)
  {
    machine.prepare(velocity_mps, duration, msg_rate_hz);
  }

}