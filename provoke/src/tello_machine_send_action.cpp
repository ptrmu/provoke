
#include "tello_dispatch.hpp"
#include "tello_machine_action.hpp"

namespace provoke
{
  namespace tello_machine_send_action
  {
    class Machine : public TelloMachineAction
    {
      const double timeout_sec_;

      Result _validate_args(YamlArgs &args, std::string &action_str, double &timeout_sec)
      {
        (void) this; // silence static warning

        // Set defaults:
        action_str.clear();
        timeout_sec = timeout_sec_;

        // Get the timeout argument if it exists
        std::string duration_str;
        args.get_arg_str("timeout", duration_str);
        if (!duration_str.empty()) {
          timeout_sec = std::strtod(duration_str.c_str(), nullptr);
        }

        // get the action argument if it exists
        args.get_arg_str("action", action_str);
        if (action_str.empty()) {
          args.get_arg_str("", action_str);
        }

        // Must have an action string.
        if (action_str.empty()) {
          return Result::make_result(ResultCodes::parse_error,
                                     "%s:%s must have an 'action' argument",
                                     dispatch_.name_.c_str(), name_.c_str());
        }

        return Result::success();
      }

    public:
      explicit Machine(TelloDispatch &dispatch, double timeout_sec) :
        TelloMachineAction{"tello_machine_send_action", dispatch},
        timeout_sec_{timeout_sec}
      {}

      ~Machine() override = default;

      Result validate_args(YamlArgs &args) override
      {
        std::string action_str;
        double timeout_sec;

        return _validate_args(args, action_str, timeout_sec);
      }

      Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
      {
        std::string action_str;
        double timeout_sec;

        auto result = _validate_args(args, action_str, timeout_sec);
        if (!result.succeeded()) {
          return result;
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Prepare %s:%s with action:'%s', timeout:%7.3f (sec)",
                    dispatch_.name_.c_str(), name_.c_str(),
                    action_str.c_str(), timeout_sec);

        return prepare_from_action_str(now, action_str, timeout_sec);
      }
    };

    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch)
    {
      return std::unique_ptr<TelloInterface>{std::make_unique<Machine>(dispatch, 10.0)};
    }
  }
}
