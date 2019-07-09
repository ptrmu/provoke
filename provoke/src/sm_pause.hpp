#ifndef SM_PAUSE_HPP
#define SM_PAUSE_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "state_machine_interface.hpp"

namespace provoke
{
  namespace sm_pause
  {
    class Machine;

    // ==============================================================================
    // Hub class
    // ==============================================================================

    class Hub
    {
      Machine &machine_;

    public:
      Hub(Machine &machine) :
        machine_{machine}
      {}

      SMResult prepare(rclcpp::Duration duration);

      SMResult set_ready(rclcpp::Duration duration);

      SMResult set_waiting(rclcpp::Time end_time);
    };

    // ==============================================================================
    // Ready state
    // ==============================================================================

    class Ready : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;
      rclcpp::Duration duration_{0, 0};

    public:
      Ready(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "ready"), impl_(impl), hub_(hub)
      {}

      SMResult prepare(rclcpp::Duration duration)
      {
        duration_ = duration;
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        auto end_time = now + duration_;
        return hub_.set_waiting(end_time);
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

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "waiting"), impl_(impl), hub_(hub)
      {}

      SMResult prepare(rclcpp::Time end_time)
      {
        end_time_ = end_time;
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        return now < end_time_ ? SMResult::success() : SMResult::conclusion();
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
      SMResult _validate_args(const StateMachineArgs &args, rclcpp::Duration &duration);

    public:
      Hub hub_;
      Ready ready_;
      Waiting waiting_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "sm_pause"}, hub_{*this}, ready_{impl, hub_},
          waiting_{impl, hub_}
      {}

      ~Machine() = default;

      SMResult validate_args(const StateMachineArgs &args) override;

      SMResult prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_PAUSE_HPP
