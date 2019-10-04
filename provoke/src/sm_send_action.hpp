#ifndef SM_SEND_ACTION_HPP
#define SM_SEND_ACTION_HPP

#include <memory>

#include "provoke_node_impl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine_interface.hpp"
#include "tello_msgs/srv/tello_action.hpp"

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

      rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;

    public:
      const std::string action_;
      double &timeout_sec_;

      rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr action_client_;
      tello_msgs::msg::TelloResponse::SharedPtr tello_response_;

      Hub(Machine &machine, std::string action, double &timeout_sec);

      SMResult sm_prepare();

      SMResult set_ready(rclcpp::Duration timeout);

      SMResult set_waiting(const rclcpp::Time timeout_time);
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

      SMResult on_timer(const rclcpp::Time &now) override
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
      using ActionFuture = std::shared_future<tello_msgs::srv::TelloAction::Response::SharedPtr>;

      Hub &hub_;
      rclcpp::Time timeout_time_{};
      ActionFuture action_future_{};

    public:
      Waiting(StateMachineInterface &machine, provoke::ProvokeNodeImpl &impl, Hub &hub) :
        StateInterface{"waiting", machine, impl}, hub_{hub}
      {}

      SMResult prepare(const rclcpp::Time timeout_time)
      {
        timeout_time_ = timeout_time;

        // Make the request and save the future.
        auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
        request->cmd = hub_.action_;
        action_future_ = hub_.action_client_->async_send_request(request);

        // Clear out the response. This will be assigned by the callback.
        hub_.tello_response_.reset();

        return SMResult::success();
      }

      SMResult on_timer(const rclcpp::Time &now) override;

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
