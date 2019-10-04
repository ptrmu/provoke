
#ifndef TELLO_DISPATCH_HPP
#define TELLO_DISPATCH_HPP

#include "shared_dispatch.hpp"
#include "tello_interface.hpp"
#include "tello_msgs/srv/tello_action.hpp"

namespace provoke
{

  class TelloDispatch : public SharedDispatch<TelloInterface>
  {
//    std::shared_future<tello_msgs::srv::TelloAction::Response::SharedPtr> action_future_{};
    rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;

  public:
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr action_client_;

    TelloDispatch(ProvokeNodeImpl &impl, int inst_index);

    ~TelloDispatch() override;

    bool is_action_client_ready();
  };

  namespace tello_machine_takeoff
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch);
  }

  namespace tello_machine_land
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch);
  }

  namespace tello_machine_send_action
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch);
  }
}
#endif //TELLO_DISPATCH_HPP
