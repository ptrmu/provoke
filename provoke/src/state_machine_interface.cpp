
#include "state_machine_interface.hpp"

#include <stdio.h>
#include <stdarg.h> // for va_list, va_start

#include "rclcpp/rclcpp.hpp"
#include "provoke_node_impl.hpp"

namespace provoke
{
  SMResult SMResult::make_result(SMResultCodes code, const std::string fmt_str, ...)
  {
    constexpr size_t string_reserve = 32;
    size_t str_len = std::max(fmt_str.size(), string_reserve);
    std::string str;


    do {
      va_list ap;
      va_start(ap, fmt_str); // NOTE: vsnprintf modifies ap so it has to be initialized in the loop

      str.resize(str_len);

      auto final_n = vsnprintf(const_cast<char *>(str.data()), str_len, fmt_str.c_str(), ap);

      // For an encoding error just return what is in the buffer.
      if (final_n < 0) {
        break;
      }

      // If the buffer sufficient, resize it and finish.
      if (final_n < static_cast<int>(str_len)) {
        str.resize(final_n + 1); // don't truncate the trailing null!
        break;
      }

      // The buffer was not large enough. So resize it.
      str_len = final_n + 1;

      va_end(ap);
    } while (true);

    return SMResult{code, str};
  }

  SMResult StateInterface::on_timer(rclcpp::Time now)
  {
    (void) now;
    return SMResult::make_result(SMResultCodes::logic_error,
                                 "State:%s has not implemented 'on_timer()'",
                                 name_.c_str());
  }

  SMResult StateInterface::on_tello_response(tello_msgs::msg::TelloResponse *msg)
  {
    (void) msg;
    return SMResult::make_result(SMResultCodes::logic_error,
                                 "State:%s has not implemented 'on_tello_response()'",
                                 name_.c_str());
  }

  SMResult StateMachineInterface::validate_args(const StateMachineArgs &args)
  {
    (void) args;
    return SMResult::make_result(SMResultCodes::logic_error,
                                 "Machine:%s has not implemented 'validate_args()'",
                                 name_.c_str());
  }

  SMResult StateMachineInterface::prepare_from_args(const StateMachineArgs &args)
  {
    (void) args;
    return SMResult::make_result(SMResultCodes::logic_error,
                                 "Machine:%s has not implemented 'prepare_from_args()'",
                                 name_.c_str());
  }
}
