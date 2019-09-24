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
      explicit Hub(Machine &machine) :
        machine_{machine}
      {}

      SMResult sm_prepare(rclcpp::Duration duration);

      SMResult set_ready(rclcpp::Duration duration);

      SMResult set_waiting(const rclcpp::Time end_time);
    };

    // ==============================================================================
    // Ready state
    // ==============================================================================

    class Ready : public provoke::StateInterface
    {
      Hub &hub_;
      rclcpp::Duration duration_{0, 0};

    public:
      Ready(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"ready", machine,  impl}, hub_{hub}
      {}

      SMResult prepare(rclcpp::Duration duration)
      {
        duration_ = duration;
        return SMResult::success();
      }

      SMResult on_timer(const rclcpp::Time &now) override
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
      Hub &hub_;
      rclcpp::Time end_time_;

    public:
      Waiting(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"waiting", machine,  impl}, hub_{hub}
      {}

      SMResult prepare(const rclcpp::Time end_time)
      {
        end_time_ = end_time;
        return SMResult::success();
      }

      SMResult on_timer(const rclcpp::Time &now) override
      {
        return now < end_time_ ? SMResult::success() : SMResult::conclusion();
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
      Hub hub_;

      SMResult _validate_args(const StateMachineArgs &args, rclcpp::Duration &duration);

    public:
      Ready ready_;
      Waiting waiting_;

      explicit Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{"sm_pause", impl}, hub_{*this}, ready_{*this, impl, hub_},
          waiting_{*this, impl, hub_}
      {}

      ~Machine() override = default;

      SMResult validate_args(const StateMachineArgs &args) override;

      SMResult prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_PAUSE_HPP
