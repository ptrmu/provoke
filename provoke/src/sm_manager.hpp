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

    class YamlParser;

#define PK1 "\n- pause: 1.5\n- pause: 0.5\n- pause: 2.5\n- pause: 1"
#define PK2 "\n- takeoff\n- land"
#define PK3 "\n- takeoff\n- pause: 1.5\n- land"
#define PK4 "\n- takeoff\n- out_back: {vel_x: 0.0, vel_x: 1.0, vel_x: 0.1, dur_out: 1.5, dur_back: 0.5, hz: 5}\n- land"

#define SM_MANAGER_ALL_PARAMS \
  CXT_MACRO_MEMBER(poke_list_go, int, 1) /* poke list to execute */\
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

      std::map<std::string, StateMachineInterface *> sm_map_{};

      SMResult validate_sm_args(std::vector<std::string> &poke_name_list,
                                std::vector<StateMachineInterface::StateMachineArgs> &poke_args_list);

      void validate_parameters();

    public:
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
      SM_MANAGER_ALL_PARAMS

      std::array<std::string const *, 4> poke_lists_{
        &poke_list_1_,
        &poke_list_2_,
        &poke_list_3_,
        &poke_list_4_,
      };
      std::array<bool, std::tuple_size<decltype(poke_lists_)>::value> poke_list_valids_;

      explicit Hub(Machine &machine);

      ~Hub();

      StateMachineInterface *find_state_machine(std::string &poke_name);

      SMResult sm_prepare();

      SMResult set_running(const std::string &poke_list);

      SMResult set_complete();
    };

    // ==============================================================================
    // Running state
    // ==============================================================================

    // Note: This is a state in the base state machine. It should keep running
    // and never return false.
    class Running : public provoke::StateInterface
    {
      Hub &hub_;
      std::vector<std::string> poke_name_list_{};
      std::vector<StateMachineInterface::StateMachineArgs> poke_args_list_{};

      int current_poke_idx_{};
      StateMachineInterface *current_sm_poke_{};

      SMResult is_done(SMResult res)
      {
        if (!res.succeeded()) {
          if (!prepare_sm_poke(current_poke_idx_ + 1).succeeded()) {
            hub_.set_complete();
          }
        }
        return SMResult::success(); // always return success from the manager state machine
      }

    public:
      Running(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"running", machine,  impl}, hub_{hub}
      {}

      SMResult prepare_sm_poke(int idx);

      SMResult prepare(const std::string &poke_list);

      SMResult on_timer(rclcpp::Time now) override
      {
        return is_done(current_sm_poke_->state().on_timer(now));
      }

      SMResult on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        return is_done(current_sm_poke_->state().on_tello_response(msg));
      }
    };

    // ==============================================================================
    // Complete state
    // ==============================================================================

    // Note: This is a state in the base state machine. It should keep running
    // and never return false.
    class Complete : public provoke::StateInterface
    {
      Hub &hub_;

    public:
      Complete(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"complete", machine,  impl}, hub_{hub}
      {}

      SMResult prepare()
      {
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        (void) now;

        // test to see if the parameter poke_list_go is non-zero. If it
        // is then we will load and execute the indicated poke_list.
        if (hub_.poke_list_go_ == 0) {
          return SMResult::success();
        }

        // The poke_lists are numbered starting from 1, but they are indexed starting from 0.
        auto poke_list_go = hub_.poke_list_go_ - 1;

        if (poke_list_go < 0 || static_cast<size_t>(poke_list_go) >= hub_.poke_lists_.size()) {
          hub_.poke_list_go_ = 0; // ToDo: set parameter on node as well
          return SMResult::success();
        }

        if (!hub_.poke_list_valids_[poke_list_go]) {
          hub_.poke_list_go_ = 0;
          return SMResult::success();
        }

        hub_.set_running(*hub_.poke_lists_[poke_list_go]);
        hub_.poke_list_go_ = 0;
        return SMResult::success();
      }

      SMResult on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        (void) msg;
        return SMResult::success();
      }
    };

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public StateMachineInterface
    {
    public:
      Hub hub_;
      Running running_;
      Complete complete_;

      explicit Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{"sm_manager", impl}, hub_{*this}, running_{*this, impl, hub_},
          complete_{*this, impl, hub_}
      {}

      ~Machine() override = default;
    };
  }
}
#endif //SM_MANAGER_HPP
