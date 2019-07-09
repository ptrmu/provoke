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

      SMResult sm_prepare(rclcpp::Duration timeout);

      SMResult set_ready(rclcpp::Duration timeout);

      SMResult set_waiting(rclcpp::Time timeout_time);
    };

    // ==============================================================================
    // Ready state
    // ==============================================================================

    class Ready : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;
      rclcpp::Duration timeout_{0, 0};

    public:
      Ready(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "ready"), impl_(impl), hub_(hub)
      {}

      SMResult prepare(rclcpp::Duration timeout)
      {
        timeout_ = timeout;
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        // make the async client call.
        auto timeout_time = now + timeout_;
        return hub_.set_waiting(timeout_time);
      }
    };

    // ==============================================================================
    // Waiting state
    // ==============================================================================

    class Waiting : public provoke::StateInterface
    {
      provoke::ProvokeNodeImpl &impl_;
      Hub &hub_;
      rclcpp::Time timeout_time_{};

    public:
      Waiting(provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface(impl, "waiting"), impl_(impl), hub_(hub)
      {}

      SMResult prepare(rclcpp::Time timeout_time)
      {
        timeout_time_ = timeout_time;
        return SMResult::success();
      }

      SMResult on_timer(rclcpp::Time now) override
      {
        // If the timeout time has been exceeded, then we have a failure
        // and the manager should stop processing.
        if (now > timeout_time_) {
          return SMResult{SMResultCodes::failure, "send_action timed out."};
        }

        // Check the future. If it hasn't returned, then return success.
        // If it had no error, also return success. If it had a problem, then
        // return failure.
        return SMResult::success();
      }

      SMResult on_tello_response(tello_msgs::msg::TelloResponse *msg) override
      {
        (void) msg;
        // Check the response, and return conclusion if there was no error, otherwise
        // return an error.
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
        : StateMachineInterface{impl, "sm_send_action"}, hub_{*this, action}, ready_{impl, hub_},
          waiting_{impl, hub_}
      {}

      ~Machine() = default;

      SMResult validate_args(const StateMachineArgs &args) override;

      SMResult prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_SEND_ACTION_HPP
