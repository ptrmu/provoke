
#include "timer_interface.hpp"

namespace provoke
{
  Result TimerInterface::validate_args(YamlArgs &args)
  {
    (void) args;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'validate_args()'",
                               name_.c_str());
  }

  Result TimerInterface::prepare_from_args(YamlArgs &args)
  {
    (void) args;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'prepare_from_args()'",
                               name_.c_str());
  }
}

