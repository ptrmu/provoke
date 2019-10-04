
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

    // Make the request.
    dispatch_.send_tello_action_request(action_str);

    pending_result_ = Result::success();

    return Result::success();

  }

  void TelloMachineAction::set_concluded()
  {
    pending_result_ = Result::conclusion();
  }

  Result TelloMachineAction::on_timer(const rclcpp::Time &now)
  {
    // Only pay attention to this message if the pending_result is success.
    if (!pending_result_.succeeded()) {
      return pending_result_;
    }

    // If the timeout time has been exceeded, then we have a failure
    // and the manager should stop processing.
    if (now >= timeout_time_) {
      return Result::make_result(ResultCodes::timeout, "%s:%s timed out.",
                                 dispatch_.name_.c_str(), name_.c_str());
    }

    return pending_result_;
  }

  Result TelloMachineAction::on_tello_response(const tello_msgs::msg::TelloResponse &response)
  {
    // Only pay attention to this message if the pending_result is success.
    if (!pending_result_.succeeded()) {
      return pending_result_;
    }

    // Now that a response has been received we are done with this action.
    set_concluded();

//      RCLCPP_INFO(impl_.node_.get_logger(),
//                  "Received tello_response - rc:%d, until timeout:%7.3f",
//                  tello_response_->rc, (timeout_time_ - now).seconds());

    // Figure out what if an error occurred.
    if (response.rc == response.OK) {
      return Result::success();

    } else if (response.rc == response.ERROR) {
      return Result::make_result(ResultCodes::failure,
                                 "%s action %s failed with error %s",
                                 name_.c_str(), action_str_.c_str(), response.str.c_str());

    } else if (response.rc == response.TIMEOUT) {
      return Result::make_result(ResultCodes::failure,
                                 "%s action %s timed out with error %s",
                                 name_.c_str(), action_str_.c_str(), response.str.c_str());
    }

    return Result::make_result(ResultCodes::failure,
                               "%s action %s failed with unknown error %s",
                               name_.c_str(), action_str_.c_str(), response.str.c_str());
  }

  Result TelloMachineAction::on_tello_action_response(const tello_msgs::srv::TelloAction_Response &action_response)
  {
    // Only pay attention to this message if the pending_result is success.
    if (!pending_result_.succeeded()) {
      return pending_result_;
    }

    RCLCPP_INFO(impl_.node_.get_logger(),
                "%s future complete - rc:%d",
                name_.c_str(), action_response.rc);

    if (action_response.rc == action_response.OK) {
      return Result::success();

    } else if (action_response.rc == action_response.ERROR_BUSY) {
      return Result::make_result(ResultCodes::failure,
                                 "%s action %s returned busy.",
                                 name_.c_str(), action_str_.c_str());

    } else if (action_response.rc == action_response.ERROR_NOT_CONNECTED) {
      return Result::make_result(ResultCodes::failure,
                                 "%s action %s returned not connected.",
                                 name_.c_str(), action_str_.c_str());
    }

    return Result::make_result(ResultCodes::failure,
                               "%s action %s returned an unknown error: %d.",
                               name_.c_str(), action_str_.c_str(), action_response.rc);
  }
}