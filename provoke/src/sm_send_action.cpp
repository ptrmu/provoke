
#include "sm_send_action.hpp"

#include "tello_msgs/msg/tello_response.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    Hub::Hub(Machine &machine, std::string action, double &timeout_sec) :
      machine_{machine}, action_{std::move(action)}, timeout_sec_{timeout_sec}
    {
      auto &node = machine_.impl_.node_;

      action_client_ = node.create_client<tello_msgs::srv::TelloAction>("tello_action");

      tello_response_sub_ = node.create_subscription<tello_msgs::msg::TelloResponse>(
        "tello_response",
        rclcpp::ServicesQoS(),
        [this](tello_msgs::msg::TelloResponse::SharedPtr msg) -> void
        {
          tello_response_ = msg;
        });
    }

    SMResult Hub::sm_prepare()
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s(%s) timeout:%7.3f",
                  machine_.name_.c_str(), action_.c_str(), timeout_sec_);

      rclcpp::Duration timeout{std::chrono::milliseconds(static_cast<int>(timeout_sec_ * 1000))};
      return set_ready(timeout);
    }

    SMResult Hub::set_ready(rclcpp::Duration timeout)
    {
      auto res = machine_.ready_.prepare(timeout);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.ready_);
    }

    SMResult Hub::set_waiting(const rclcpp::Time timeout_time)
    {
      auto res = machine_.waiting_.prepare(timeout_time);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.waiting_);
    }

    SMResult Waiting::on_timer(const rclcpp::Time &now)
    {
      // If the timeout time has been exceeded, then we have a failure
      // and the manager should stop processing.
      if (now > timeout_time_) {
        return SMResult{SMResultCodes::failure, "send_action timed out."};
      }

      // Check the future. If it hasn't returned, then continue and check the response.
      // If it had no error, also return success. If it had a problem, then
      // return failure.
      if (action_future_.valid() &&
          action_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {

        auto response = action_future_.get();
        RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                    "checking on future - rc:%d",
                    response->rc);

        if (response->rc == response->ERROR_BUSY) {
          return SMResult::make_result(SMResultCodes::failure,
                                       "%s action %s returned busy.",
                                       name_.c_str(), hub_.action_.c_str());

        } else if (response->rc == response->ERROR_NOT_CONNECTED) {
          return SMResult::make_result(SMResultCodes::failure,
                                       "%s action %s returned not connected.",
                                       name_.c_str(), hub_.action_.c_str());

        } else if (response->rc != response->OK) {
          return SMResult::make_result(SMResultCodes::failure,
                                       "%s action %s returned an unknown error: %d.",
                                       name_.c_str(), hub_.action_.c_str(), response->rc);
        }

        // The future returned Ok. So set a flag so we don't do this test anymore.
        // And then fall into the code checkeds for the response.
        action_future_ = ActionFuture{};
      }

      // Check to see if the tello_response message has been received.
      if (hub_.tello_response_ != nullptr) {

        RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                    "checking on tello_response - rc:%d, until timeout:%7.3f",
                    hub_.tello_response_->rc, (timeout_time_ - now).seconds());

        if (hub_.tello_response_->rc == hub_.tello_response_->OK) {
          return SMResult::conclusion();

        } else if (hub_.tello_response_->rc == hub_.tello_response_->ERROR) {
          SMResult::make_result(SMResultCodes::failure,
                                "%s action %s failed with error %s",
                                name_.c_str(), hub_.action_.c_str(), hub_.tello_response_->str.c_str());

        } else if (hub_.tello_response_->rc == hub_.tello_response_->TIMEOUT) {
          SMResult::make_result(SMResultCodes::failure,
                                "%s action %s timed out with error %s",
                                name_.c_str(), hub_.action_.c_str(), hub_.tello_response_->str.c_str());

        } else {
          SMResult::make_result(SMResultCodes::failure,
                                "%s action %s failed with unknown error %s",
                                name_.c_str(), hub_.action_.c_str(), hub_.tello_response_->str.c_str());
        }
      }

      // No termination conditions have been discovered so wait for
      // the next on_timer event.
      return SMResult::success();
    }

    SMResult Machine::_validate_args(const StateMachineInterface::StateMachineArgs &args)
    {
      if (!args.empty()) {
        return SMResult::make_result(SMResultCodes::failure,
                                     "Machine '%s' requires no arguments. %d were passed.",
                                     name_.c_str(), args.size());
      }
      return SMResult::success();
    }

    SMResult Machine::validate_args(const StateMachineArgs &args)
    {
      return _validate_args(args);
    }

    SMResult Machine::prepare_from_args(const StateMachineArgs &args)
    {
      auto res = _validate_args(args);
      if (!res.succeeded()) {
        return res;
      }
      return hub_.sm_prepare();
    }
  }

  std::unique_ptr<sm_send_action::Machine>
  sm_send_action_factory(provoke::ProvokeNodeImpl &impl, const std::string &action, double &timeout_sec)
  {
    return std::make_unique<sm_send_action::Machine>(impl, action, timeout_sec);
  }
}