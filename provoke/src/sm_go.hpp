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
      Hub(Machine &machine) :
        machine_{machine}
      {}

      SMResult sm_prepare(tf2::Vector3 velocity_mps, rclcpp::Duration duration, double msg_rate_hz);

      SMResult set_ready();

      SMResult set_waiting(rclcpp::Time end_time, rclcpp::Time next_msg_time, rclcpp::Duration inter_msg_duration);

      void send_go();
    };

    // ==============================================================================
    // Ready state
    // ==============================================================================

    class Ready : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;
      rclcpp::Duration duration_{0, 0};
      rclcpp::Duration inter_msg_duration_{0, 0};

    public:
      Ready(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "ready"), impl_(impl), hub_(hub)
      {}

      SMResult prepare(rclcpp::Duration duration, double msg_rate_hz)
      {
        duration_ = duration;
        auto milliseconds = msg_rate_hz == 0.0 ?
                            std::chrono::milliseconds::zero() :
                            std::chrono::milliseconds{int(1000.0 / msg_rate_hz)};
        inter_msg_duration_ = milliseconds == std::chrono::milliseconds::zero() ?
                              rclcpp::Duration{std::chrono::milliseconds::zero()} :
                              rclcpp::Duration(milliseconds);

        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        hub_.send_go();

        auto end_time = now + duration_;
        auto next_msg_time = inter_msg_duration_ == std::chrono::milliseconds::zero() ?
                             rclcpp::Time{} : now + inter_msg_duration_;

        hub_.set_waiting(end_time, next_msg_time, inter_msg_duration_);

        return SMResult::success();
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
      rclcpp::Duration inter_msg_duration_{0, 0};

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "waiting"), impl_(impl), hub_(hub)
      {}

      SMResult prepare(rclcpp::Time end_time, rclcpp::Time next_msg_time, rclcpp::Duration inter_msg_duration)
      {
        end_time_ = end_time;
        next_msg_time_ = next_msg_time;
        inter_msg_duration_ = inter_msg_duration;
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        if (now >= end_time_) {
          return SMResult::failure();
        }

        if (next_msg_time_.nanoseconds() != 0 && now >= next_msg_time_) {
          hub_.send_go();
          while (now >= next_msg_time_) {
            next_msg_time_ = next_msg_time_ + inter_msg_duration_;
          }
        }

        return SMResult::success();
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
    public:
      Hub hub_;
      Ready ready_;
      Waiting waiting_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "sm_go"}, hub_{*this}, ready_{impl, hub_},
          waiting_{impl, hub_}
      {}

      ~Machine() = default;
    };
  }
}
#endif //SM_GO_HPP
