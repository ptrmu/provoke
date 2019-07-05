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

      std::unique_ptr<sm_out_back::Machine> sm_out_back_;

    public:
      Hub(Machine &machine);

      ~Hub();

      void prepare(const std::string &poke_name);

      void set_state_a(StateMachineInterface *sub_machine);

      void set_complete();
    };

    // ==============================================================================
    // StateA state
    // ==============================================================================

    class StateA : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;

      StateMachineInterface *sub_machine_;

      bool is_done(bool ret)
      {
        if (ret) {
          return true;
        }

        hub_.set_complete();
        return false;
      }

    public:
      StateA(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "state_a"), impl_(impl), hub_(hub)
      {}

      void prepare(StateMachineInterface *sub_machine)
      {
        sub_machine_ = sub_machine;
      }

      bool on_timer(rclcpp::Time now) override
      {
        return is_done(sub_machine_->state().on_timer(now));
      }

      bool on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        return is_done(sub_machine_->state().on_tello_response(msg));
      }
    };

    // ==============================================================================
    // Complete state
    // ==============================================================================

    class Complete : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;

    public:
      Complete(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "complete"), impl_(impl), hub_(hub)
      {}

      void prepare()
      {
        RCLCPP_INFO(impl_.node_.get_logger(), "Entering Complete state");
      }

      bool on_timer(rclcpp::Time now) override
      {
        (void) now;
        return false;
      }

      bool on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        (void) msg;
        return false;
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
    public:
      Hub hub_;
      StateA state_a_;
      Complete complete_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "sm_manager"}, hub_{*this}, state_a_{impl, hub_}, complete_{impl, hub_}
      {}

      ~Machine() = default;
    };
  }
}
#endif //SM_MANAGER_HPP
