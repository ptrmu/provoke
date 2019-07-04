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
        StateInterface(impl, "Ready"), impl_(impl), hub_(hub)
      {}

      void prepare(rclcpp::Duration duration)
      {
        duration = duration_;
      }

      virtual bool on_timer(rclcpp::Time now) override
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
        StateInterface(impl, "Waiting"), impl_(impl), hub_(hub)
      {}

      void prepare(rclcpp::Time end_time)
      {
        end_time_ = end_time;
      }

      virtual bool on_timer(rclcpp::Time now) override
      {
        return now < end_time_;
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

      void prepare(rclcpp::Duration duration)
      {
        hub_.set_ready(duration);
      }
    };
  }
}
#endif //SM_PAUSE_HPP
