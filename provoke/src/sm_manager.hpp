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
    // PrepareAndSet class
    // ==============================================================================

    class PrepareAndSet
    {
      Machine &machine_;

    public:
      PrepareAndSet(Machine &machine) :
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
      PrepareAndSet &prepare_and_set_;

    public:
      StateA(provoke::ProvokeNodeImpl &impl, PrepareAndSet &prepare_and_set) :
        StateInterface(impl, "StateA"), impl_(impl), prepare_and_set_(prepare_and_set)
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
      PrepareAndSet prepare_and_set_;

    public:
      StateA state_a_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "Send Action"}, prepare_and_set_{*this}, state_a_{impl, prepare_and_set_}
      {}

      ~Machine() = default;

      void on_enter() override
      {
      }
    };
  }
}
#endif //SM_MANAGER_HPP
