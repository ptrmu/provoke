
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace timer_machine_pause
  {
    enum class States
    {
      ready = 0,
      waiting,
      timeout,
      logic_error,
      failure,
    };

    class Machine : public TimerInterface
    {
      TimerDispatch &dispatch_;

    public:
      explicit Machine(TimerDispatch &dispatch)
        : TimerInterface{"timer_machine_pause", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine() override = default;

      Result on_timer(rclcpp::Time now) override
      {
        (void) now;
        return Result::failure();
      }

      Result _validate_args(YamlArgs &args, rclcpp::Duration &duration)
      {
        std::string duration_str;

        auto result = args.get_arg_str("dur", duration_str);
        if (duration_str.empty()) {

          result = args.get_arg_str("", duration_str);
          if (duration_str.empty()) {
            duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(1 * 1000)));
            return Result::success();
          }
        }

        auto secs = std::strtod(duration_str.c_str(), nullptr);
        duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(secs * 1000)));
        return Result::success();
      }


      Result validate_args(YamlArgs &args) override
      {
        rclcpp::Duration duration{0, 0};
        return _validate_args(args, duration);
      }

      Result prepare_from_args(YamlArgs &args) override
      {
        (void) args;
        return Result::failure();
      }
    };

    std::unique_ptr<TimerInterface> factory(TimerDispatch &dispatch)
    {
      return std::unique_ptr<TimerInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

