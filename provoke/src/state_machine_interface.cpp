
#include "state_machine_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "provoke_node_impl.hpp"

namespace provoke
{
  SMResult StateInterface::on_timer(rclcpp::Time now)
  {
    (void) now;
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_timer()' not overridden for state %s", name_.c_str());
    return SMResult{SMResultCodes::failure, "on_timer un-implemented"};
  }

  SMResult StateInterface::on_tello_response(tello_msgs::msg::TelloResponse *msg)
  {
    (void) msg;
    RCLCPP_ERROR(impl_.node_.get_logger(), "Action 'on_tello_response()' not overridden for state %s", name_.c_str());
    return SMResult{SMResultCodes::failure, "on_tello_response un-implemented"};
  }

  SMResult StateMachineInterface::validate_args(const StateMachineArgs &args)
  {
    (void) args;
    return SMResult{SMResultCodes::failure, "validate_args un-implemented"};
  }

  SMResult StateMachineInterface::prepare_from_args(const StateMachineArgs &args)
  {
    (void) args;
    return SMResult{SMResultCodes::failure, "prepare_from_args un-implemented"};
  }

  /*
#include <string>
#include <cstdarg>
#include <cstdlib>
#include <memory>
#include <algorithm>

inline std::string string_format2(size_t string_reserve, const std::string fmt_str, ...)
{
    size_t str_len = (std::max)(fmt_str.size(), string_reserve);
    std::string str;

    va_list ap;
    va_start(ap, fmt_str);

    while (true) {
        str.resize(str_len);

        const int final_n = vsnprintf(const_cast<char *>(str.data()), str_len, fmt_str.c_str(), ap);

        if (final_n < 0 || final_n >= int(str_len))
            str_len += (std::abs)(final_n - int(str_len) + 1);
        else {
            str.resize(final_n); // do not forget to shrink the size!
            break;
        }
    }

    va_end(ap);

    return str;
}
*/
}
