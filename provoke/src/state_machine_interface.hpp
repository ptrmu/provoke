#ifndef STATE_MACHINE_INTERFACE_HPP
#define STATE_MACHINE_INTERFACE_HPP

#include <string>

namespace rclcpp
{
  class Node;
}

namespace provoke
{
  class ProvokeNodeImpl;

  class StateInterface
  {
  public:
    ProvokeNodeImpl &impl_;
    std::string name_;

    StateInterface(ProvokeNodeImpl &impl_, std::string name);

    virtual ~StateInterface() = default;

    virtual void on_enter();

    virtual bool on_timer();
  };


  class StateMachineInterface
  {
  public:
    StateInterface *state_;

    virtual ~StateMachineInterface() = default;

    void set_state(StateInterface &state)
    {
      state_ = &state;
      state.on_enter();
    }
  };
}
#endif //STATE_MACHINE_INTERFACE_HPP
