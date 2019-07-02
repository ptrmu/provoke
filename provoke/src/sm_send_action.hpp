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
    // PrepareAndSet class
    // ==============================================================================

    class PrepareAndSet
    {
      Machine &machine_;

    public:
      PrepareAndSet(Machine &machine) :
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
      PrepareAndSet &prepare_and_set_;

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, PrepareAndSet &prepare_and_set) :
        StateInterface(impl, "Waiting"), impl_(impl), prepare_and_set_(prepare_and_set)
      {}

      virtual bool on_timer(rclcpp::Time now) override
      {
        (void)now;
        prepare_and_set_.ready(3);
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
      PrepareAndSet &prepare_and_set_;

    public:
      Ready(provoke::ProvokeNodeImpl &impl, PrepareAndSet &prepare_and_set) :
        StateInterface(impl, "Ready"), impl_(impl), prepare_and_set_(prepare_and_set)
      {}

      void prepare(int i)
      {
        (void) i;
      }

      virtual bool on_timer(rclcpp::Time now) override
      {
        (void)now;        prepare_and_set_.waiting();
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
      Ready ready_;
      Waiting waiting_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "Send Action"}, prepare_and_set_{*this}, ready_{impl, prepare_and_set_}, waiting_{impl, prepare_and_set_}
      {}

      ~Machine() = default;

      void on_enter() override
      {
      }
    };
  }
}
#endif //SM_SEND_ACTION_HPP
