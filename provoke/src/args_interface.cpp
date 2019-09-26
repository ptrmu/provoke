
#include "args_interface.hpp"

#include "yaml_args.hpp"

namespace provoke
{
  Result ArgsInterface::validate_args(YamlArgs &args)
  {
    (void) args;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'validate_args()'",
                               name_.c_str());
  }

  Result ArgsInterface::prepare_from_args(const rclcpp::Time &now, YamlArgs &args)
  {
    (void) now;
    (void) args;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'prepare_from_args()'",
                               name_.c_str());
  }
}

