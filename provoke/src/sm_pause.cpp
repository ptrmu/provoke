
#include "sm_pause.hpp"

namespace provoke
{
  namespace sm_pause
  {
    void Hub::prepare(rclcpp::Duration duration)
    {
      RCLCPP_DEBUG(machine_.impl_.node_.get_logger(),
                   "Prepare sm:%s (duration:%7.3f)",
                   machine_.name_, duration.seconds());
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
      (void) duration;
      std::ostringstream oss{};
      if (args.size() != 1) {
        oss << "send_action takes no arguments.";
        return oss.str();
      }
      /*
       * // stod example
#include <iostream>   // std::cout
#include <string>     // std::string, std::stod

int main ()
{
  std::string orbits ("365.24 29.53");
  std::string::size_type sz;     // alias of size_t

  double earth = std::stod (orbits,&sz);
  double moon = std::stod (orbits.substr(sz));
  std::cout << "The moon completes " << (earth/moon) << " orbits per Earth year.\n";
  return 0;
       */
      return std::string{};
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