
#include "tello_dispatch.hpp"
#include "tello_machine_action.hpp"

namespace provoke
{
  class TelloMachineConstAction : public TelloMachineAction
  {
    const std::string action_str_;
    const double timeout_sec_;


    Result _validate_args(YamlArgs &args, double &timeout_sec)
    {
      (void) this; // silence static warning

      // Set defaults:
      timeout_sec = timeout_sec_;

      // Get the timeout argument if it exists
      std::string duration_str;
      args.get_arg_str("timeout", duration_str);
      if (duration_str.empty()) {
        args.get_arg_str("", duration_str);
      }
      if (!duration_str.empty()) {
        timeout_sec = std::strtod(duration_str.c_str(), nullptr);
      }

      return Result::success();
    }

  public:
    explicit TelloMachineConstAction(TelloDispatch &dispatch, const std::string &action_str, double timeout_sec) :
      TelloMachineAction{std::string("tello_machine_").append((action_str)), dispatch},
      action_str_{action_str},timeout_sec_{timeout_sec}
    {}

    ~TelloMachineConstAction() override = default;

    Result validate_args(YamlArgs &args) override
    {
      double timeout_sec;
      return _validate_args(args, timeout_sec);
    }

    Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
    {
      double timeout_sec;
      _validate_args(args, timeout_sec);

      RCLCPP_INFO(impl_.node_.get_logger(),
                  "Prepare %s:%s timeout:%7.3f (sec)",
                  dispatch_.name_.c_str(), name_.c_str(),
                  timeout_sec);

      return prepare_from_action_str(now, action_str_, timeout_sec);
    }
  };

  namespace tello_machine_takeoff
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch)
    {
      return std::unique_ptr<TelloInterface>{std::make_unique<TelloMachineConstAction>(dispatch, "takeoff", 10.0)};
    }
  }

  namespace tello_machine_land
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch)
    {
      return std::unique_ptr<TelloInterface>{std::make_unique<TelloMachineConstAction>(dispatch, "land", 10.0)};
    }
  }
}
