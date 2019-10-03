
#include "tello_machine_action.hpp"

#include "tello_dispatch.hpp"

namespace provoke
{
  TelloMachineAction::TelloMachineAction(std::string name, TelloDispatch &dispatch)
    : TelloInterface{std::move(name), dispatch.impl_}, dispatch_{dispatch}
  {}

  TelloMachineAction::~TelloMachineAction() = default;

  Result TelloMachineAction::prepare_from_action_str(const rclcpp::Time &now,
                                                     const std::string &action_str,
                                                     double timeout_sec)
  {
    rclcpp::Duration timeout{std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000))};
    timeout_time_ = now + timeout;
    action_str_ = action_str;

    // Make the request and save the future.
    auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
    request->cmd = action_str;
    action_future_ = dispatch_.action_client_->async_send_request(request);

    // Clear out the response. This will be assigned by the callback.
    tello_response_.reset();

    concluded_ = false;
    return Result::success();

  }

  void TelloMachineAction::set_concluded()
  {
    tello_response_.reset();
    action_future_ = ActionFuture{};
    concluded_ = true;
  }

  Result TelloMachineAction::on_timer(const rclcpp::Time &now)
  {
    // If concluded, return conclusion.
    if (concluded_) {
      return Result::conclusion();
    }

    // If the timeout time has been exceeded, then we have a failure
    // and the manager should stop processing.
    if (now >= timeout_time_) {
      set_concluded();
      return Result::make_result(ResultCodes::timeout, "%s:%s timed out.",
                                 dispatch_.name_.c_str(), name_.c_str());
    }

    // Check the future. If it hasn't returned, then continue and check the response.
    // If it had no error, also return success. If it had a problem, then
    // return failure.
    if (action_future_.valid() &&
        action_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {

      auto response = action_future_.get();
      RCLCPP_INFO(impl_.node_.get_logger(),
                  "future complete - rc:%d",
                  response->rc);

      if (response->rc == response->ERROR_BUSY) {
        return Result::make_result(ResultCodes::failure,
                                   "%s action %s returned busy.",
                                   name_.c_str(), action_str_.c_str());

      } else if (response->rc == response->ERROR_NOT_CONNECTED) {
        return Result::make_result(ResultCodes::failure,
                                   "%s action %s returned not connected.",
                                   name_.c_str(), action_str_.c_str());

      } else if (response->rc != response->OK) {
        return Result::make_result(ResultCodes::failure,
                                   "%s action %s returned an unknown error: %d.",
                                   name_.c_str(), action_str_.c_str(), response->rc);
      }

      // The future returned Ok. So set a flag so we don't do this test anymore.
      // And then fall into the code checking for the response.
      action_future_ = ActionFuture{};
    }

    // Check to see if the tello_response message has been received.
    if (tello_response_ != nullptr) {

//      RCLCPP_INFO(impl_.node_.get_logger(),
//                  "checking on tello_response - rc:%d, until timeout:%7.3f",
//                  tello_response_->rc, (timeout_time_ - now).seconds());

      if (tello_response_->rc == tello_response_->OK) {
        set_concluded();
        return Result::conclusion();

      } else if (tello_response_->rc == tello_response_->ERROR) {
        Result::make_result(ResultCodes::failure,
                            "%s action %s failed with error %s",
                            name_.c_str(), action_str_.c_str(), tello_response_->str.c_str());

      } else if (tello_response_->rc == tello_response_->TIMEOUT) {
        Result::make_result(ResultCodes::failure,
                            "%s action %s timed out with error %s",
                            name_.c_str(), action_str_.c_str(), tello_response_->str.c_str());

      } else {
        Result::make_result(ResultCodes::failure,
                            "%s action %s failed with unknown error %s",
                            name_.c_str(), action_str_.c_str(), tello_response_->str.c_str());
      }
    }

    // No termination conditions have been discovered so wait for
    // the next on_timer event.
    return Result::success();
  }

  Result TelloMachineAction::on_tello_response(tello_msgs::msg::TelloResponse::SharedPtr &msg)
  {
    if (!concluded_) {
      tello_response_ = msg;
    }
    return Result::success();
  }
}