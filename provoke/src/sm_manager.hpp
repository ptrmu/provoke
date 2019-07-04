#ifndef SM_MANAGER_HPP
#define SM_MANAGER_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "state_machine_interface.hpp"

namespace provoke
{
  namespace sm_manager
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

      void state_a();
    };

    // ==============================================================================
    // StateA state
    // ==============================================================================

    class StateA : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;

    public:
      StateA(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "StateA"), impl_(impl), hub_(hub)
      {}

      virtual bool on_timer(rclcpp::Time now) override
      {
        (void)now;
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
      StateA state_a_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "Send Action"}, hub_{*this}, state_a_{impl, hub_}
      {}

      ~Machine() = default;
    };
  }
}
#endif //SM_MANAGER_HPP
