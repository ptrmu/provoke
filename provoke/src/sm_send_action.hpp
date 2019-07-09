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
      double &timeout_sec_;

      Hub(Machine &machine, std::string action, double &timeout_sec) :
        machine_{machine}, action_{std::move(action)}, timeout_sec_{timeout_sec}
      {}

      SMResult sm_prepare();

      SMResult set_ready(rclcpp::Duration timeout);

      SMResult set_waiting(rclcpp::Time timeout_time);
    };

    // ==============================================================================
    // Ready state
    // ==============================================================================

    class Ready : public provoke::StateInterface
    {
      Hub &hub_;
      rclcpp::Duration timeout_{0, 0};

    public:
      Ready(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"ready", machine, impl}, hub_{hub}
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
      Hub &hub_;
      rclcpp::Time timeout_time_{};

    public:
      Waiting(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"waiting", machine, impl}, hub_{hub}
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
      Hub hub_;

      SMResult _validate_args(const StateMachineInterface::StateMachineArgs &args);

    public:
      Ready ready_;
      Waiting waiting_;

      explicit Machine(provoke::ProvokeNodeImpl &impl, const std::string &action, double &timeout_sec)
        : StateMachineInterface{"sm_send_action", impl}, hub_{*this, action, timeout_sec}, ready_{*this, impl, hub_},
          waiting_{*this, impl, hub_}
      {}

      ~Machine() override = default;

      SMResult validate_args(const StateMachineArgs &args) override;

      SMResult prepare_from_args(const StateMachineArgs &args) override;
    };
  }
}
#endif //SM_SEND_ACTION_HPP
