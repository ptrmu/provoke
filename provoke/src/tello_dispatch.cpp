
#include "tello_dispatch.hpp"

namespace provoke
{
  TelloDispatch::TelloDispatch(ProvokeNodeImpl &impl, int inst_index) :
    SharedDispatch("tello_dispatch", impl, inst_index)
  {
    auto &node = impl_.node_;

    action_client_ = node.create_client<tello_msgs::srv::TelloAction>("tello_action");

    // Subscribe to the "tello_responsee" messages.
    // TODO: pick up the message name from the node parameters.
    tello_response_sub_ = node.create_subscription<tello_msgs::msg::TelloResponse>(
      "tello_response",
      rclcpp::ServicesQoS(),
      [this](tello_msgs::msg::TelloResponse::SharedPtr msg) -> void
      {
        // Dispatch any tello_response messages that we receive.
        if (running_machine_ != nullptr) {
          auto result = running_machine_->on_tello_response(msg);
          if (!result.succeeded()) {
            RCLCPP_ERROR(impl_.node_.get_logger(),
                         "on_tello_response() failed for machine %s with error: %s",
                         running_machine_->name_.c_str(), result.msg().c_str());
          }
        }
      });
    (void) tello_response_sub_;

    new_machine_ = [this](const std::string &cmd) -> std::unique_ptr<TelloInterface>
    {
      if (cmd =="takeoff") {
        return tello_machine_takeoff::factory(*this);
      } else if (cmd =="land") {
        return tello_machine_land::factory(*this);
      }
      return std::unique_ptr<TelloInterface>{};
    };
  }

  TelloDispatch::~TelloDispatch() = default;
}
