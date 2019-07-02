#ifndef STATE_MACHINE_INTERFACE_HPP
#define STATE_MACHINE_INTERFACE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/tello_response.hpp"

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

    virtual void on_enter();

    virtual bool on_timer(rclcpp::Time now);

    virtual bool on_tello_response(tello_msgs::msg::TelloResponse * msg);
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
      state.on_enter();
    }

    virtual void on_enter();
  };

  std::unique_ptr<StateMachineInterface> sm_send_action_factory(ProvokeNodeImpl &impl);

  std::unique_ptr<StateMachineInterface> sm_manager_factory(ProvokeNodeImpl &impl);

}
#endif //STATE_MACHINE_INTERFACE_HPP
