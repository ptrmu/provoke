
#ifndef TIMER_INTERFACE_HPP
#define TIMER_INTERFACE_HPP

#include "base_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "result.hpp"

namespace provoke
{
  class YamlArgs;

  class TimerInterface : public BaseInterface
  {
  public:
    TimerInterface(std::string name, ProvokeNodeImpl &impl) :
      BaseInterface{std::move(name), impl}
    {}

    TimerInterface() = delete;

    virtual ~TimerInterface() = default;

    virtual Result validate_args(YamlArgs &args);

    virtual Result prepare_from_args(YamlArgs &args);

    Result validate_cmd_seq(YamlArgs &args,
                            std::function<std::unique_ptr<TimerInterface>(const std::string &)> get_machine);
  };
}

#endif //TIMER_INTERFACE_HPP
