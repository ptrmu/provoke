
#include "sm_pause.hpp"

namespace provoke
{
  namespace sm_pause
  {
    void Hub::prepare(rclcpp::Duration duration)
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s (duration:%7.3f sec.)",
                  machine_.name_.c_str(), duration.seconds());
      set_ready(duration);
    }

    void Hub::set_ready(rclcpp::Duration duration)
    {
      machine_.ready_.prepare(duration);
      machine_.set_state(machine_.ready_);
    }

    void Hub::set_waiting(rclcpp::Time end_time)
    {
      machine_.waiting_.prepare(end_time);
      machine_.set_state(machine_.waiting_);
    }

    std::string Machine::_validate_args(const StateMachineArgs &args, rclcpp::Duration &duration)
    {
      std::ostringstream oss{};
      if (args.size() != 1) {
        oss << "sm_pause takes 1 argument.";
      }

      auto pair = args.begin();
      char *next;
      auto secs = std::strtod(pair->second.c_str(), &next);
      duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(secs * 1000)));

      return oss.str();
    }

    std::string Machine::validate_args(const StateMachineArgs &args)
    {
      rclcpp::Duration duration{0, 0};
      return _validate_args(args, duration);
    }

    void Machine::prepare_from_args(const StateMachineArgs &args)
    {
      rclcpp::Duration duration{0, 0};
      if (_validate_args(args, duration).empty()) {
        hub_.prepare(duration);
      }
    }
  }

  std::unique_ptr<sm_pause::Machine> sm_pause_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_pause::Machine>(impl);
  }

  void sm_prepare(sm_pause::Machine &machine, rclcpp::Duration duration)
  {
    machine.hub_.prepare(duration);
  }

}