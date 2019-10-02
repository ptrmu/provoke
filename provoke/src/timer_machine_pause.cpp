
#include "provoke_node_impl.hpp"
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace timer_machine_pause
  {
    class Machine : public ArgsInterface
    {
      TimerDispatch &dispatch_;

      rclcpp::Time end_time_{};

    public:
      explicit Machine(TimerDispatch &dispatch)
        : ArgsInterface{"timer_machine_pause", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine() override = default;

      Result on_timer(const rclcpp::Time &now) override
      {
        // If we haven't reached the end time, then continue running.
        if (now < end_time_) {
          return Result::success();
        }

        // Time has expired
        return Result::conclusion();
      }

      Result _validate_args(YamlArgs &args, rclcpp::Duration &duration)
      {
        (void)this; // silence could be static warning

        std::string duration_str;

        args.get_arg_str("dur", duration_str);
        if (duration_str.empty()) {

          args.get_arg_str("", duration_str);
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

      Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
      {
        rclcpp::Duration duration{0, 0};
        auto result = _validate_args(args, duration);
        if (!result.succeeded()) {
          return result;
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Prepare %s:%s (duration:%7.3f sec.)",
                    dispatch_.name_.c_str(), name_.c_str(), duration.seconds());

        end_time_ = now + duration;
        return Result::success();
      }
    };

    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch)
    {
      return std::unique_ptr<ArgsInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

