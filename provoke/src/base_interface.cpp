
#include "base_interface.hpp"

namespace provoke
{
  Result BaseInterface::on_timer(rclcpp::Time now)
  {
    (void) now;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'on_timer()'",
                               name_.c_str());
  }
}
