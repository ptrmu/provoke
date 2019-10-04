
#ifndef TELLO_MACHINE_ACTION_HPP
#define TELLO_MACHINE_ACTION_HPP

#include <future>

#include "rclcpp/time.hpp"
#include "tello_interface.hpp"
#include "tello_msgs/srv/tello_action.hpp"


namespace provoke
{
  class TelloDispatch;

  class TelloMachineAction : public TelloInterface
  {
    using ActionFuture = std::shared_future<tello_msgs::srv::TelloAction::Response::SharedPtr>;

    Result pending_result_{Result::conclusion()};
    std::string action_str_{};
    rclcpp::Time timeout_time_{};

  protected:
    TelloDispatch &dispatch_;

    Result prepare_from_action_str(const rclcpp::Time &now, const std::string &action_str, double timeout_sec);

    void set_concluded();

  public:
    explicit TelloMachineAction(std::string name, TelloDispatch &dispatch);

    ~TelloMachineAction() override;

    Result on_timer(const rclcpp::Time &now) override;

    Result on_tello_response(const tello_msgs::msg::TelloResponse &response) override;

    Result on_tello_action_response(const tello_msgs::srv::TelloAction_Response &action_response) override;
  };
}

#endif //TELLO_MACHINE_ACTION_HPP
