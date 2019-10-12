
#ifndef TELLO_DISPATCH_HPP
#define TELLO_DISPATCH_HPP

#include "shared_dispatch.hpp"
#include "tello_interface.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include "tello_msgs/msg/flight_data.hpp"


namespace provoke
{

  class TelloDispatch : public SharedDispatch<TelloInterface>
  {
    using ActionFuture = std::shared_future<tello_msgs::srv::TelloAction::Response::SharedPtr>;

    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr action_client_;
    rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;
    rclcpp::Subscription<tello_msgs::msg::FlightData >::SharedPtr flight_data_sub_;

    ActionFuture action_future_{};

  public:
    TelloDispatch(ProvokeNodeImpl &impl, int inst_index, const std::string &ns);

    ~TelloDispatch() override;

    bool is_action_client_ready();

    void send_tello_action_request(const std::string &action_str);
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
