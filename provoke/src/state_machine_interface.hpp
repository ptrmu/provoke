#ifndef STATE_MACHINE_INTERFACE_HPP
#define STATE_MACHINE_INTERFACE_HPP

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace provoke
{
  class ProvokeNodeImpl;

  class StateMachineInterface;

  enum class SMResultCodes
  {
    success = 0,
    conclusion,
    timeout,
    logic_error,
    failure,
  };

  class SMResult
  {
    SMResultCodes code_;
    std::string msg_;

  public:
    SMResult() :
      code_{SMResultCodes::success}, msg_{}
    {
    }

    SMResult(SMResultCodes code, const std::string &msg) :
      code_{code}, msg_{msg}
    {}

    static SMResult make_result(SMResultCodes code, std::string fmt_str, ...);

    auto code()
    { return code_; }

    auto &msg()
    { return msg_; }

    bool succeeded()
    { return code_ == SMResultCodes::success; }

    static SMResult success()
    {
      return SMResult{};
    }

    static SMResult conclusion()
    {
      return SMResult{SMResultCodes::conclusion, ""};
    }

    static SMResult failure()
    {
      return SMResult{SMResultCodes::failure, ""};
    }
  };

  class StateInterface
  {
  public:
    StateMachineInterface &machine_;
    ProvokeNodeImpl &impl_;
    std::string name_;

    StateInterface(std::string name, StateMachineInterface &machine, ProvokeNodeImpl &impl);

    virtual ~StateInterface() = default;

    virtual SMResult on_timer(rclcpp::Time now);

    virtual SMResult on_tello_response(tello_msgs::msg::TelloResponse *msg);

    bool is_active_state();
  };


  class StateMachineInterface
  {
    StateInterface *state_{};

  public:
    using StateMachineArgs = std::map<std::string, std::string>;

    std::string name_;
    ProvokeNodeImpl &impl_;

    StateMachineInterface(std::string name, ProvokeNodeImpl &impl) :
      name_{std::move(name)}, impl_{impl}
    {}

    StateMachineInterface() = delete;

    virtual ~StateMachineInterface() = default;

    StateInterface &state()
    {
      return *state_;
    }

    SMResult set_state(StateInterface &state)
    {
      state_ = &state;
      return SMResult::success();
    }

    virtual SMResult validate_args(const StateMachineArgs &args);

    virtual SMResult prepare_from_args(const StateMachineArgs &args);
  };

  namespace sm_manager
  {
    class Machine;
  }

  std::unique_ptr<sm_manager::Machine> sm_manager_factory(ProvokeNodeImpl &impl);

  namespace sm_send_action
  {
    class Machine;
  }

  std::unique_ptr<sm_send_action::Machine>
  sm_send_action_factory(ProvokeNodeImpl &impl, const std::string &action, double &timeout_sec);

  namespace sm_pause
  {
    class Machine;
  }

  std::unique_ptr<sm_pause::Machine> sm_pause_factory(ProvokeNodeImpl &impl);

  namespace sm_go
  {
    class Machine;
  }

  std::unique_ptr<sm_go::Machine> sm_go_factory(ProvokeNodeImpl &impl);

  namespace sm_out_back
  {
    class Machine;
  }

  std::unique_ptr<sm_out_back::Machine> sm_out_back_factory(ProvokeNodeImpl &impl);
}
#endif //STATE_MACHINE_INTERFACE_HPP
