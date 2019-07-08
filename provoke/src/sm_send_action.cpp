
#include "sm_send_action.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    void Hub::prepare()
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
        "Prepare sm:%s(%s)",
        machine_.name_, action_);
      set_ready();
    }

    void Hub::set_ready()
    {
      machine_.ready_.prepare();
      machine_.set_state(machine_.ready_);
    }

    void Hub::set_waiting()
    {
      machine_.set_state(machine_.waiting_);
    }

    static std::string _validate_args(const StateMachineInterface::StateMachineArgs &args)
    {
      if (!args.empty()) {
        std::ostringstream oss{};
        oss << "send_action takes no arguments.";
        return oss.str();
      }
      return std::string{};
    }

    std::string Machine::validate_args(const StateMachineArgs &args)
    {
      return _validate_args(args);
    }

    void Machine::prepare_from_args(const StateMachineArgs &args)
    {
      if (_validate_args(args).empty()) {
        hub_.prepare();
      }
    }
  }

  std::unique_ptr<sm_send_action::Machine>
  sm_send_action_factory(provoke::ProvokeNodeImpl &impl, const std::string &action)
  {
    return std::make_unique<sm_send_action::Machine>(impl, action);
  }
}