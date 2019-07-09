
#include "sm_pause.hpp"

namespace provoke
{
  namespace sm_pause
  {
    SMResult Hub::sm_prepare(rclcpp::Duration duration)
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s (duration:%7.3f sec.)",
                  machine_.name_.c_str(), duration.seconds());
      return set_ready(duration);
    }

    SMResult Hub::set_ready(rclcpp::Duration duration)
    {
      auto res = machine_.ready_.prepare(duration);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.ready_);
    }

    SMResult Hub::set_waiting(rclcpp::Time end_time)
    {
      auto res = machine_.waiting_.prepare(end_time);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.waiting_);
    }

    SMResult Machine::_validate_args(const StateMachineArgs &args, rclcpp::Duration &duration)
    {
      std::ostringstream oss{};
      if (args.size() != 1) {
        oss << name_ << " takes a single argument.";
      }

      auto pair = args.begin();
      char *next;
      auto secs = std::strtod(pair->second.c_str(), &next);
      duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(secs * 1000)));

      return oss.str().empty() ? SMResult::success() : SMResult{SMResultCodes::failure, oss.str()};
    }

    SMResult Machine::validate_args(const StateMachineArgs &args)
    {
      rclcpp::Duration duration{0, 0};
      return _validate_args(args, duration);
    }

    SMResult Machine::prepare_from_args(const StateMachineArgs &args)
    {
      rclcpp::Duration duration{0, 0};
      auto res = _validate_args(args, duration);
      if (!res.succeeded()) {
        return res;
      }
      return hub_.sm_prepare(duration);
    }
  }

  std::unique_ptr<sm_pause::Machine> sm_pause_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_pause::Machine>(impl);
  }
}