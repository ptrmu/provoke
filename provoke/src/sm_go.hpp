#ifndef SM_GO_HPP
#define SM_GO_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "state_machine_interface.hpp"

namespace provoke
{
  namespace sm_go
  {
    class Machine;

    // ==============================================================================
    // Hub class
    // ==============================================================================

    class Hub
    {
      Machine &machine_;
      tf2::Vector3 velocity_mps_;

    public:
      rclcpp::Duration duration_{0, 0};
      rclcpp::Duration inter_msg_duration_{0, 0};

      Hub(Machine &machine) :
        machine_{machine}
      {}

      void set_ready();

      void set_waiting(rclcpp::Time end_time, rclcpp::Time next_msg_time);

      void prepare(tf2::Vector3 velocity_mps, rclcpp::Duration duration, double msg_rate_hz)
      {
        velocity_mps_ = velocity_mps;
        duration_ = duration;
        auto milliseconds = msg_rate_hz == 0.0 ?
                            std::chrono::milliseconds::zero() :
                            std::chrono::milliseconds{int(1000.0 / msg_rate_hz)};
        inter_msg_duration_ = milliseconds == std::chrono::milliseconds::zero() ?
                              rclcpp::Duration{std::chrono::milliseconds::zero()} :
                              rclcpp::Duration(milliseconds);
      }

      void send_go()
      {

      }
    };

    // ==============================================================================
    // Ready state
    // ==============================================================================

    class Ready : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;

    public:
      Ready(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "Ready"), impl_(impl), hub_(hub)
      {}

      virtual bool on_timer(rclcpp::Time now) override
      {
        hub_.send_go();

        auto end_time = now + hub_.duration_;
        auto next_msg_time = hub_.inter_msg_duration_ == std::chrono::milliseconds::zero() ?
                             rclcpp::Time{} : now + hub_.inter_msg_duration_;
        hub_.set_waiting(end_time, next_msg_time);
        return true;
      }
    };

    // ==============================================================================
    // Waiting state
    // ==============================================================================

    class Waiting : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;
      rclcpp::Time end_time_;
      rclcpp::Time next_msg_time_;

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "Waiting"), impl_(impl), hub_(hub)
      {}

      void prepare(rclcpp::Time end_time, rclcpp::Time next_msg_time)
      {
        end_time_ = end_time;
        next_msg_time_ = next_msg_time;
      }

      virtual bool on_timer(rclcpp::Time now) override
      {
        if (now >= end_time_)
        {
          return false;
        }

        if (next_msg_time_.nanoseconds() != 0 && now >= next_msg_time_)
        {
          hub_.send_go();
          while (now >= next_msg_time_)
          next_msg_time_ = next_msg_time_ + hub_.inter_msg_duration_;
        }

        return true;
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
      Hub hub_;

    public:
      Ready ready_;
      Waiting waiting_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "Send Action"}, hub_{*this}, ready_{impl, hub_},
          waiting_{impl, hub_}
      {}

      ~Machine() = default;

      void prepare(tf2::Vector3 velocity_mps, rclcpp::Duration duration, double msg_rate_hz)
      {
        hub_.prepare(velocity_mps, duration, msg_rate_hz);
        hub_.set_ready();
      }
    };
  }
}
#endif //SM_GO_HPP