
#include "timer_interface.hpp"
#include "provoke_node_impl.hpp"
#include "ros2_shared/context_macros.hpp"
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace base_machine
  {

    // ==============================================================================
    // Parameters
    // ==============================================================================

#define PK0 "[pause: {dur: 5}, pause: {duration: 3}, pause: 3, pause]"
#define PK1 "[par: [[pause: 2], [pause: 3, pause: 4]]]"
#define PK2 "[pause: {dur: 1}]"
#define PK3 "[tello: [takeoff, land]]"
#define PK4 "[ par: [[tello: [takeoff, land]], [tello: [takeoff, land]]]]"

#define BASE_MACHINE_ALL_PARAMS \
  CXT_MACRO_MEMBER(cmds_go, int, 4) /* poke list to execute */\
  CXT_MACRO_MEMBER(cmds_0, std::string, PK0) /* Sequence of commands 0 */ \
  CXT_MACRO_MEMBER(cmds_1, std::string, PK1) /* Sequence of commands 1 */ \
  CXT_MACRO_MEMBER(cmds_2, std::string, PK2) /* Sequence of commands 2 */ \
  CXT_MACRO_MEMBER(cmds_3, std::string, PK3) /* Sequence of commands 3 */ \
  CXT_MACRO_MEMBER(cmds_4, std::string, PK4) /* Sequence of commands 4 */ \
  /* End of list */


    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public TimerInterface
    {
      enum class States
      {
        ready = 0,
        running,
      };

      States state_{States::ready};

      int cmds_go_last_{-1};

      void validate_parameters()
      {
      }

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
      BASE_MACHINE_ALL_PARAMS

    public:
      explicit Machine(provoke::ProvokeNodeImpl &impl)
        : TimerInterface{"timer_pause", impl}
      {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(impl_.node_, (*this), n, t, d)
        CXT_MACRO_INIT_PARAMETERS(BASE_MACHINE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)
        CXT_MACRO_REGISTER_PARAMETERS_CHANGED(impl_.node_, BASE_MACHINE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, impl_.node_.get_logger(), (*this), n, t, d)
        BASE_MACHINE_ALL_PARAMS
      }

      ~Machine() override = default;

      Result on_timer_ready(const rclcpp::Time &now)
      {
        // test to see if the parameter cmds_go_ is non-negative. If it
        // is then we will load and execute the indicated cmds.
        if (cmds_go_ < 0) {
          return Result::success();
        }

        // Pick one of the lists of commands to execute. Make a copy
        // of the string because it needs to stay around while it is being
        // executed. The parameter itself could change during excution.
        std::string cmds;
        cmds_go_last_ = cmds_go_;
        switch (cmds_go_) {
          case 0:
            cmds = cmds_0_;
            break;
          case 1:
            cmds = cmds_1_;
            break;
          case 2:
            cmds = cmds_2_;
            break;
          case 3:
            cmds = cmds_3_;
            break;
          case 4:
            cmds = cmds_4_;
            break;
        }

        // We have picked up this command, so reset the go parameter.
        CXT_MACRO_SET_PARAMETER(impl_.node_, (*this), cmds_go, -1);

        // Error if out of range
        if (cmds.empty()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "cmds_go value of %d is out of range.",
                       cmds_go_last_);
          return Result::success();
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Preparing to execute cmds_%d",
                    cmds_go_last_);

        // Prepare to parse the command list.
        auto running_yaml_holder = std::make_shared<YamlHolder>();
        auto result = running_yaml_holder->from_string(cmds.c_str());

        // If the initial parse failed, report this and exit
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Parse failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Get a YamlArgs.
        std::unique_ptr<YamlArgs> yaml_args;
        result = running_yaml_holder->get_args(running_yaml_holder, yaml_args);

        // If the we couldn't get args, report this and exit
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "get_args failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Validate the cmds.
        result = impl_.timer_dispatch_->validate_args(*yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Validate failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Prepare for execution
        result = impl_.timer_dispatch_->prepare_from_args(now, *yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Prepare failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Do one on_timer() call for this cmds list just to get it started earlier.
        // In the timer dispatch we do this so that there is not a delay between
        // the end of one cmd and the start of the next.
        result = impl_.timer_dispatch_->on_timer(now);
        if (!result.succeeded()) {
          if (!result.concluded())
            RCLCPP_ERROR(impl_.node_.get_logger(),
                         "First on_timer() failed for cmds_%d with error: %s",
                         cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Transition to running state
        state_ = States::running;
        return Result::success();
      }

      Result on_timer_running(const rclcpp::Time &now)
      {
        // Dispatch the on_timer call.
        auto result = impl_.timer_dispatch_->on_timer(now);

        // If success, then just return.
        if (result.succeeded()) {
          return result.success();
        }

        // Transition to the ready state if concluded or error
        state_ = States::ready;

        // If an error occurred, then log the error.
        if (!result.concluded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "on_timer() failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "cmds_%d concluded successfully",
                    cmds_go_last_);
        return Result::failure();
      }

      Result on_timer(const rclcpp::Time &now) override
      {
        switch (state_) {
          case States::ready:
            return on_timer_ready(now);

          case States::running:
            return on_timer_running(now);

          default:
            state_ = States::ready;
            return Result::success();
        }
      }
    };

    std::unique_ptr<TimerInterface> factory(ProvokeNodeImpl &impl)
    {
      return std::unique_ptr<TimerInterface>{std::make_unique<Machine>(impl)};
    }
  }
}
