
#include "sm_go.hpp"

namespace provoke
{
  namespace sm_go
  {
    void Hub::set_ready()
    {
      machine_.set_state(machine_.ready_);
    }

    void Hub::prepare(tf2::Vector3 velocity_mps, rclcpp::Duration duration, double msg_rate_hz)
    {
      velocity_mps_ = velocity_mps;
      machine_.ready_.prepare(duration, msg_rate_hz);
      set_ready();
    }

    void Hub::set_waiting(rclcpp::Time end_time, rclcpp::Time next_msg_time, rclcpp::Duration inter_msg_duration)
    {
      machine_.waiting_.prepare(end_time, next_msg_time, inter_msg_duration);
      machine_.set_state(machine_.waiting_);
    }

    void Hub::send_go()
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(), "Send Go: x:%7.3f, y:%7.3f, z:%7.3f",
                  velocity_mps_.x(), velocity_mps_.y(), velocity_mps_.z());
    }
  }

  std::unique_ptr<sm_go::Machine> sm_go_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_go::Machine>(impl);
  }

  void sm_prepare(sm_go::Machine &machine, tf2::Vector3 velocity_mps,
                  rclcpp::Duration duration, double msg_rate_hz)
  {
    machine.hub_.prepare(velocity_mps, duration, msg_rate_hz);
  }

}