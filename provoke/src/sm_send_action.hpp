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
      Hub(Machine &machine) :
        machine_{machine}
      {}

      void ready(int i);

      void waiting();
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
        StateInterface(impl, "Waiting"), impl_(impl), hub_(hub)
      {}

      virtual bool on_timer(rclcpp::Time now) override
      {
        (void)now;
        hub_.ready(3);
        return false;
      }

      virtual bool on_tello_response(tello_msgs::msg::TelloResponse * msg) override
      {
        (void)msg;
        return false;
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

      void prepare(int i)
      {
        (void) i;
      }

      virtual bool on_timer(rclcpp::Time now) override
      {
        (void)now;        hub_.waiting();
        return false;
      }

      virtual bool on_tello_response(tello_msgs::msg::TelloResponse * msg) override
      {
        (void)msg;
        return false;
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
        : StateMachineInterface{impl, "Send Action"}, hub_{*this}, ready_{impl, hub_}, waiting_{impl, hub_}
      {}

      ~Machine() = default;
    };
  }
}
#endif //SM_SEND_ACTION_HPP
