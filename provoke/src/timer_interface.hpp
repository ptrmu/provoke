
#ifndef TIMER_INTERFACE_HPP
#define TIMER_INTERFACE_HPP

#include <memory>

#include "result.hpp"

namespace rclcpp
{
  class Time;
}

namespace provoke
{
  class ProvokeNodeImpl;

  class TimerInterface
  {
  public:
    const std::string name_;
    ProvokeNodeImpl &impl_;

    TimerInterface(std::string name, ProvokeNodeImpl &impl) :
      name_{std::move(name)}, impl_{impl}
    {}

    TimerInterface() = delete;

    virtual ~TimerInterface() = default;

    virtual Result on_timer(const rclcpp::Time &now);
  };

  namespace base_machine
  {
    std::unique_ptr<TimerInterface> factory(ProvokeNodeImpl &impl);
  }
}
#endif //TIMER_INTERFACE_HPP
