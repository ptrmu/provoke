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

      void prepare(rclcpp::Duration duration)
      {
        set_ready(duration);
      }

      void set_ready(rclcpp::Duration duration);

      void set_waiting(rclcpp::Time end_time);
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

      void prepare(rclcpp::Duration duration)
      {
        duration = duration_;
      }

      bool on_timer(rclcpp::Time now) override
      {
        auto end_time = now + duration_;
        hub_.set_waiting(end_time);
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

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "waiting"), impl_(impl), hub_(hub)
      {}

      void prepare(rclcpp::Time end_time)
      {
        end_time_ = end_time;
      }

      bool on_timer(rclcpp::Time now) override
      {
        return now < end_time_;
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
      std::string _validate_args(const StateMachineArgs &args,  rclcpp::Duration &duration);

    public:
      Hub hub_;
      Ready ready_;
      Waiting waiting_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "sm_pause"}, hub_{*this}, ready_{impl, hub_},
          waiting_{impl, hub_}
      {}

      ~Machine() = default;

      std::string validate_args(const StateMachineArgs &args) override;

      void prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_PAUSE_HPP
