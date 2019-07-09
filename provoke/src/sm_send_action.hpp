#ifndef SM_SEND_ACTION_HPP
#define SM_SEND_ACTION_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "state_machine_interface.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    class Machine;

    // ==============================================================================
    // Hub class
    // ==============================================================================

    class Hub
    {
      Machine &machine_;

    public:
      const std::string action_;

      Hub(Machine &machine, const std::string &action) :
        machine_{machine}, action_{action}
      {}

      SMResult prepare();

      SMResult set_ready();

      SMResult set_waiting();
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
        StateInterface(impl, "ready"), impl_(impl), hub_(hub)
      {}

      SMResult prepare()
      {
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        (void) now;
        hub_.set_waiting();
        return SMResult::conclusion();
      }

      SMResult on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        (void) msg;
        return SMResult::conclusion();
      }
    };

    // ==============================================================================
    // Waiting state
    // ==============================================================================

    class Waiting : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "waiting"), impl_(impl), hub_(hub)
      {}

      SMResult on_timer(rclcpp::Time now) override
      {
        (void) now;
        hub_.set_ready();
        return SMResult::conclusion();
      }

      SMResult on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        (void) msg;
        return SMResult::conclusion();
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

      Machine(provoke::ProvokeNodeImpl &impl, const std::string &action)
        : StateMachineInterface{impl, "sm_send_action"}, hub_{*this, action}, ready_{impl, hub_}, waiting_{impl, hub_}
      {}

      ~Machine() = default;

      SMResult validate_args(const StateMachineArgs &args) override;

      SMResult prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_SEND_ACTION_HPP
