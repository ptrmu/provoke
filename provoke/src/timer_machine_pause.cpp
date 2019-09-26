
#include "provoke_node_impl.hpp"
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace timer_machine_pause
  {
    enum class States
    {
      concluded = 0,
      waiting,
      running,
    };

    class Machine : public ArgsInterface
    {
      TimerDispatch &dispatch_;

      States state_{States::concluded};
      rclcpp::Duration duration_{0, 0};
      rclcpp::Time end_time_{};

    public:
      explicit Machine(TimerDispatch &dispatch)
        : ArgsInterface{"timer_machine_pause", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine() override = default;

      Result on_timer_waiting(const rclcpp::Time &now)
      {
        // Set the end time.
        end_time_ = now + duration_;
        state_ = States::running;

        // Call running state in case the timer has already expired.
        return on_timer_running(now);
      }

      Result on_timer_running(const rclcpp::Time &now)
      {
        // If we haven't reached the end time, then continue running.
        if (now <= end_time_) {
          return Result::success();
        }

        // Time has expired
        state_ = States::concluded;
        return Result::conclusion();
      }

      Result on_timer(const rclcpp::Time &now) override
      {
        switch (state_) {
          case States::waiting:
            return on_timer_waiting(now);

          case States::running:
            return on_timer_running(now);

          default:
            return Result::conclusion();
        }
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

      Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
      {
        (void)now;
        auto result = _validate_args(args, duration_);
        if (!result.succeeded()) {
          state_ = States::concluded;
          return result;
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Prepare %s:%s (duration:%7.3f sec.)",
                    dispatch_.name_.c_str(), name_.c_str(), duration_.seconds());

        state_ = States::waiting;
        return Result::success();
      }
    };

    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch)
    {
      return std::unique_ptr<ArgsInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

