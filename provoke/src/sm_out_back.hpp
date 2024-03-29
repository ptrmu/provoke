#ifndef SM_OUT_BACK_HPP
#define SM_OUT_BACK_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "sm_go.hpp"
#include "state_machine_interface.hpp"

namespace provoke
{
  namespace sm_out_back
  {
    class Machine;

    // ==============================================================================
    // Hub class
    // ==============================================================================

    class Hub
    {
      Machine &machine_;

    public:
      std::array<std::unique_ptr<sm_go::Machine>, 4> gos_;

      explicit Hub(Machine &machine);

      SMResult sm_prepare(tf2::Vector3 velocity_mps, rclcpp::Duration go_duration,
                          rclcpp::Duration stop_duration, double msg_rate_hz);

      SMResult set_running();

      SMResult set_complete();
    };

    // ==============================================================================
    // Running state
    // ==============================================================================

    class Running : public provoke::StateInterface
    {
      Hub &hub_;
      size_t go_idx_{};

    public:
      Running(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"running", machine, impl}, hub_{hub}
      {}

      SMResult prepare()
      {
        go_idx_ = 0;
        return SMResult::success();
      }

      SMResult on_timer(const rclcpp::Time &now) override
      {
        // call on_timer for this go.
        auto cont = hub_.gos_[go_idx_]->state().on_timer(now);
        if (cont.code() != SMResultCodes::conclusion) {
          return cont;
        }

        // This go state machine has completed. Move to the next one.
        go_idx_ += 1;
        if (go_idx_ < hub_.gos_.size()) {
          return SMResult::success();
        }

        // All the go state machines have completed so move to completed state
        hub_.set_complete();
        return SMResult::conclusion();
      }
    };

    // ==============================================================================
    // Complete state
    // ==============================================================================

    class Complete : public provoke::StateInterface
    {
      Hub &hub_;

    public:
      Complete(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"complete", machine, impl}, hub_{hub}
      {}

      SMResult on_timer(const rclcpp::Time &now) override
      {
        (void) now;
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
      Hub hub_;

      SMResult _validate_args(const StateMachineArgs &args, tf2::Vector3 &velocity_mps,
                              rclcpp::Duration &go_duration, rclcpp::Duration &stop_duration,
                              double &msg_rate_hz);

    public:
      Running running_;
      Complete complete_;

      explicit Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{"sm_out_back", impl}, hub_{*this}, running_{*this, impl, hub_},
          complete_{*this, impl, hub_}
      {}

      ~Machine() override = default;

      SMResult validate_args(const StateMachineArgs &args) override;

      SMResult prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_OUT_BACK_HPP
