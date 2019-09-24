
#ifndef ARGS_INTERFACE_HPP
#define ARGS_INTERFACE_HPP

#include <functional>

#include "timer_interface.hpp"
#include "result.hpp"

namespace provoke
{
  class YamlArgs;

  class YamlSeq;

  class ArgsInterface : public TimerInterface
  {
  public:
    ArgsInterface(std::string name, ProvokeNodeImpl &impl) :
      TimerInterface{std::move(name), impl}
    {}

    ArgsInterface() = delete;

    virtual ~ArgsInterface() = default;

    virtual Result validate_args(YamlArgs &args);

    virtual Result prepare_from_args(YamlArgs &args);
  };
}

#endif //ARGS_INTERFACE_HPP