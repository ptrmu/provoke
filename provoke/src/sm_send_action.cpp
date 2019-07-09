
#include "sm_send_action.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    SMResult Hub::sm_prepare()
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s(%s) timeout:%7.3f",
                  machine_.name_.c_str(), action_.c_str(), timeout_sec_);

      rclcpp::Duration timeout{std::chrono::milliseconds(static_cast<int>(timeout_sec_ * 1000))};
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

    SMResult Machine::_validate_args(const StateMachineInterface::StateMachineArgs &args)
    {
      if (!args.empty()) {
        return SMResult::make_result(SMResultCodes::failure,
                                     "Machine '%s' requires no arguments. %d were passed.",
                                     name_.c_str(), args.size());
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
      return hub_.sm_prepare();
    }
  }

  std::unique_ptr<sm_send_action::Machine>
  sm_send_action_factory(provoke::ProvokeNodeImpl &impl, const std::string &action, double &timeout_sec)
  {
    return std::make_unique<sm_send_action::Machine>(impl, action, timeout_sec);
  }
}