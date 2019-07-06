#ifndef SM_MANAGER_HPP
#define SM_MANAGER_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "ros2_shared/context_macros.hpp"
#include "state_machine_interface.hpp"

namespace provoke
{
  namespace sm_manager
  {
    class Machine;

#define PK1 "land\n"
#define PK2 "- takeoff\n- land\n"
#define PK3 "- takeoff\n- pause: 1.5\n- land\n"
#define PK4 "- takeoff\n- out_back: {vel: [0.0, 1.0, 0.1], dur_out: 1.5, dur_back: 0.5, hz: 5}\n- land\n"

#define SM_MANAGER_ALL_PARAMS \
  CXT_MACRO_MEMBER(poke_list_go, int, 0) /* poke list to execute */\
  CXT_MACRO_MEMBER(poke_list_1, std::string, PK1) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(poke_list_2, std::string, PK2) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(poke_list_3, std::string, PK3) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(poke_list_4, std::string, PK4) /* poke_list 1 */ \
  /* End of list */

    // ==============================================================================
    // Hub class
    // ==============================================================================

    class Hub
    {
      Machine &machine_;

      std::unique_ptr<sm_send_action::Machine> sm_land_;
      std::unique_ptr<sm_send_action::Machine> sm_takeoff_;
      std::unique_ptr<sm_go::Machine> sm_go_;
      std::unique_ptr<sm_pause::Machine> sm_pause_;
      std::unique_ptr<sm_out_back::Machine> sm_out_back_;

      std::map<std::string, StateMachineInterface *> sm_map{};

      void validate_parameters();

    public:
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
      SM_MANAGER_ALL_PARAMS

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

        // test to see if the parameter poke_list_go is non-zero. If it
        // is then we will load and execute the indicated poke_list.
        if (hub_.poke_list_go_ == 0) {
          return false;
        }
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
