
#include "sm_send_action.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    SMResult Hub::sm_prepare(rclcpp::Duration timeout)
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s(%s)",
                  machine_.name_.c_str(), action_.c_str());
      return set_ready(timeout);
    }

    SMResult Hub::set_ready(rclcpp::Duration timeout)
    {
      auto res = machine_.ready_.prepare(timeout);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.ready_);
    }

    SMResult Hub::set_waiting(rclcpp::Time timeout_time)
    {
      auto res = machine_.waiting_.prepare(timeout_time);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.waiting_);
    }

    static SMResult _validate_args(const StateMachineInterface::StateMachineArgs &args)
    {
      if (!args.empty()) {
        std::ostringstream oss{};
        oss << "send_action takes no arguments.";
        return SMResult{SMResultCodes::failure, oss.str()};
      }
      return SMResult::success();
    }

    SMResult Machine::validate_args(const StateMachineArgs &args)
    {
      return _validate_args(args);
    }

    SMResult Machine::prepare_from_args(const StateMachineArgs &args)
    {
      auto res = _validate_args(args);
      if (!res.succeeded()) {
        return res;
      }
      return hub_.sm_prepare(rclcpp::Duration{0, 0});
    }
  }

  std::unique_ptr<sm_send_action::Machine>
  sm_send_action_factory(provoke::ProvokeNodeImpl &impl, const std::string &action)
  {
    return std::make_unique<sm_send_action::Machine>(impl, action);
  }
}