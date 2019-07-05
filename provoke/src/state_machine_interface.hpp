#ifndef STATE_MACHINE_INTERFACE_HPP
#define STATE_MACHINE_INTERFACE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace provoke
{
  class ProvokeNodeImpl;

  class StateInterface
  {
  public:
    ProvokeNodeImpl &impl_;
    std::string name_;

    StateInterface(ProvokeNodeImpl &impl, std::string name) :
      impl_(impl), name_(name)
    {}

    virtual ~StateInterface() = default;

    virtual bool on_timer(rclcpp::Time now);

    virtual bool on_tello_response(tello_msgs::msg::TelloResponse *msg);
  };


  class StateMachineInterface
  {
    StateInterface *state_;

  public:
    ProvokeNodeImpl &impl_;
    std::string name_;

    StateMachineInterface(ProvokeNodeImpl &impl, std::string name) :
      impl_(impl), name_(name)
    {
    }

    StateMachineInterface() = delete;

    virtual ~StateMachineInterface() = default;

    StateInterface &state()
    {
      return *state_;
    }

    void set_state(StateInterface &state)
    {
      state_ = &state;
    }
  };

  namespace sm_manager
  {
    class Machine;
  }

  std::unique_ptr<sm_manager::Machine> sm_manager_factory(ProvokeNodeImpl &impl);

  void sm_prepare(sm_manager::Machine &machine, const std::string &poke_name);

  namespace sm_send_action
  {
    class Machine;
  }

  std::unique_ptr<sm_send_action::Machine> sm_send_action_factory(ProvokeNodeImpl &impl, const std::string &action);

  void sm_prepare(sm_send_action::Machine &machine);

  namespace sm_pause
  {
    class Machine;
  }

  std::unique_ptr<sm_pause::Machine> sm_pause_factory(ProvokeNodeImpl &impl);

  void sm_prepare(sm_pause::Machine &machine, rclcpp::Duration duration);

  namespace sm_go
  {
    class Machine;
  }

  std::unique_ptr<sm_go::Machine> sm_go_factory(ProvokeNodeImpl &impl);

  void sm_prepare(sm_go::Machine &machine, tf2::Vector3 velocity_mps,
                  rclcpp::Duration duration, double msg_rate_hz);

  namespace sm_out_back
  {
    class Machine;
  }

  std::unique_ptr<sm_out_back::Machine> sm_out_back_factory(ProvokeNodeImpl &impl);

  void sm_prepare(sm_out_back::Machine &machine, tf2::Vector3 velocity_mps,
                  rclcpp::Duration go_duration, rclcpp::Duration stop_duration, double msg_rate_hz);
}
#endif //STATE_MACHINE_INTERFACE_HPP
