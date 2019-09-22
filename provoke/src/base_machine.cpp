
#include "base_interface.hpp"
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

#define PK0 "pause"
#define PK1 "\n- land\n- pause: 1"
#define PK2 "\n- pause: 3\n- takeoff\n- land\n- pause: 1"
#define PK3 "\n- pause: 1\n- takeoff\n- go: {duration: 2, hz: 5}\n- go: {v_y: 0.2, duration: 5, hz: 5}\n- land\n- pause: 1"
#define PK4 "\n- pause: 3\n- takeoff\n- out_back: {v_y: 0.1, dur_out: 3, dur_back: 1, hz: 1}\n- land"

#define BASE_MACHINE_ALL_PARAMS \
  CXT_MACRO_MEMBER(cmds_go, int, 0) /* poke list to execute */\
  CXT_MACRO_MEMBER(cmds_0, std::string, PK0) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(cmds_1, std::string, PK1) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(cmds_2, std::string, PK2) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(cmds_3, std::string, PK3) /* poke_list 1 */ \
  CXT_MACRO_MEMBER(cmds_4, std::string, PK4) /* poke_list 1 */ \
  /* End of list */


    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public BaseInterface
    {
      enum class States
      {
        ready = 0,
        running = 1,
      };

      States state_{States::ready};

      int cmds_go_last_{-1};
      std::string running_cmds_{};
      std::unique_ptr<YamlArgs> running_yaml_args_{};

      void validate_parameters()
      {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, impl_.node_.get_logger(), (*this), n, t, d)
        BASE_MACHINE_ALL_PARAMS
      }

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
      BASE_MACHINE_ALL_PARAMS

    public:
      explicit Machine(provoke::ProvokeNodeImpl &impl)
        : BaseInterface{"timer_pause", impl}
      {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(impl_.node_, (*this), n, t, d)
        CXT_MACRO_INIT_PARAMETERS(BASE_MACHINE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)
        CXT_MACRO_REGISTER_PARAMETERS_CHANGED(impl_.node_, BASE_MACHINE_ALL_PARAMS, validate_parameters)
      }

      ~Machine() override = default;

      Result on_timer_ready(rclcpp::Time now)
      {
        // test to see if the parameter cmds_go_ is non-negative. If it
        // is then we will load and execute the indicated cmds.
        if (cmds_go_ < 0) {
          return Result::success();
        }

        // Pick one of the lists of commands to execute. Make a copy
        // of the string because it needs to stay around while it is being
        // executed. The parameter itself could change during excution.
        running_cmds_.clear();
        cmds_go_last_ = cmds_go_;
        switch (cmds_go_) {
          case 0:
            running_cmds_ = cmds_0_;
            break;
          case 1:
            running_cmds_ = cmds_1_;
            break;
          case 2:
            running_cmds_ = cmds_2_;
            break;
          case 3:
            running_cmds_ = cmds_3_;
            break;
          case 4:
            running_cmds_ = cmds_4_;
            break;
        }

        // Error if out of range
        if (running_cmds_.empty()) {
        }

        // We have picked up this command, so reset the go parameter.
        CXT_MACRO_SET_PARAMETER(impl_.node_, (*this), cmds_go, -1);

        // Error if out of range
        if (running_cmds_.empty()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "cmds_go value of %d is out of range.",
                       cmds_go_last_);
          return Result::success();
        }

        // Prepare to parse the command list.
        Result result;
        YamlArgs validate_yaml_args{};
        result = validate_yaml_args.from_string(running_cmds_.c_str());

        // If the initial parse failed, report this and exit
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Parse failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Validate the cmds.
        result = impl_.timer_dispatch_->validate_args(validate_yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Validate failed for cmds_%d with error: %s",
                       cmds_go_last_, result.msg().c_str());
          return Result::success();
        }

        // Have valid cmds. Create a YamlArgs for use during the execution.
        running_yaml_args_ = std::make_unique<YamlArgs>();
        result = running_yaml_args_->from_string(running_cmds_.c_str());

        // Prepare for execution
        result = impl_.timer_dispatch_->prepare_from_args(*running_yaml_args_);
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
        state_ = States::ready;
        return Result::success();
      }

      Result on_timer_running(rclcpp::Time now)
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

      Result on_timer(rclcpp::Time now) override
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

    std::unique_ptr<BaseInterface> factory(ProvokeNodeImpl &impl)
    {
      return std::unique_ptr<BaseInterface>{std::make_unique<Machine>(impl)};
    }
  }
}
