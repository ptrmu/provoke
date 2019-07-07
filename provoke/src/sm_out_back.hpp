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

      Hub(Machine &machine);

      void prepare(tf2::Vector3 velocity_mps, rclcpp::Duration go_duration, rclcpp::Duration stop_duration,
                   double msg_rate_hz)
      {
        auto velocity_back = velocity_mps * -1;
        auto velocity_stop = tf2::Vector3{};

        sm_prepare(*gos_[0], velocity_mps, go_duration, msg_rate_hz);
        sm_prepare(*gos_[1], velocity_stop, stop_duration, msg_rate_hz);
        sm_prepare(*gos_[2], velocity_back, go_duration, msg_rate_hz);
        sm_prepare(*gos_[3], velocity_stop, stop_duration, msg_rate_hz);

        set_running();
      }

      void set_running();

      void set_complete();
    };

    // ==============================================================================
    // Running state
    // ==============================================================================

    class Running : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;
      size_t go_idx_;

    public:
      Running(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "running"), impl_(impl), hub_(hub)
      {}

      void prepare()
      {
        go_idx_ = 0;
      }

      bool on_timer(rclcpp::Time now) override
      {
        // call on_timer for this go.
        auto cont = hub_.gos_[go_idx_]->state().on_timer(now);
        if (cont) {
          return true;
        }

        // This go state machine has completed. Move to the next one.
        go_idx_ += 1;
        if (go_idx_ < hub_.gos_.size()) {
          return true;
        }

        // All the go state machines have completed so move to completed state
        hub_.set_complete();
        return false;
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
      Running running_;
      Complete complete_;

      Machine(provoke::ProvokeNodeImpl &impl)
        : StateMachineInterface{impl, "sm_out_back"}, hub_{*this}, running_{impl, hub_}, complete_{impl, hub_}
      {}

      ~Machine() = default;
    };
  }
}
#endif //SM_OUT_BACK_HPP