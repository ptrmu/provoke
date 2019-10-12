
#include "timer_interface.hpp"
#include "provoke_node_impl.hpp"
#include "ros2_shared/context_macros.hpp"
#include "timer_dispatch.hpp"
#include "tf2/LinearMath/Scalar.h"
#include "yaml_args.hpp"

namespace provoke
{
  class SequenceGenerator
  {
    static std::vector<std::tuple<double, double>> generate_rotation(int revolutions, int stops_per_rev)
    {
      std::vector<std::tuple<double, double>> waypoints_2d;

      double angle_increment = TF2SIMD_2_PI / stops_per_rev;
      double angle = 0.0;

      waypoints_2d.emplace_back(std::tuple<double, double>{1.0, 0.});

      for (int r1 = 0; r1 < revolutions; r1 += 1) {
        for (int r2 = 0; r2 < stops_per_rev; r2 += 1) {
          auto back_angle = angle + angle_increment / 2 - TF2SIMD_PI;
          waypoints_2d.emplace_back(std::tuple<double, double>{std::cos(back_angle), std::sin(back_angle)});
          angle += angle_increment;
          waypoints_2d.emplace_back(std::tuple<double, double>{std::cos(angle), std::sin(angle)});
        }
      }

      return waypoints_2d;
    }

    static std::string generate_rot_oscillate_1(int radius, int revolutions, int stops_per_rev)
    {
      std::string go_commands{"[tello: [takeoff, send: go 50 0 50 50, "};

      auto rotations = generate_rotation(revolutions, stops_per_rev);
      double p0{0.0};
      double p1{0.0};
      for (const auto &r : rotations) {
        auto n0 = std::get<0>(r);
        auto n1 = std::get<1>(r);
        auto d0 = n0 - p0;
        auto d1 = n1 - p1;
        p0 = n0;
        p1 = n1;
        std::string s0{std::to_string(static_cast<int>(d0 * radius))};
        std::string s1{std::to_string(static_cast<int>(d1 * radius))};
        go_commands.append("send: go 0 ").append(s0).append(" ").append(s1).append(" 50, ");
      }

      go_commands.append("land]]");
      return go_commands;
    }

    static std::string generate_rot_oscillate_2(int radius, int revolutions, int stops_per_rev)
    {
      std::string go_commands{"[par: [[tello: [takeoff, send: go 50 0 50 50]], [tello: [takeoff, send: go 50 0 50 50]]], "};

      auto rotations = generate_rotation(revolutions, stops_per_rev);
      double p0{0.0};
      double p1{0.0};
      for (const auto &r : rotations) {
        auto n0 = std::get<0>(r);
        auto n1 = std::get<1>(r);
        auto d0 = n0 - p0;
        auto d1 = n1 - p1;
        p0 = n0;
        p1 = n1;
        std::string s0{std::to_string(static_cast<int>(d0 * radius))};
        std::string s1{std::to_string(static_cast<int>(d1 * radius))};
        std::string send{"[tello: [send: go 0 "};
        send.append(s0).append(" ").append(s1).append(" 50]]");
        go_commands.append("par: [").append(send).append(", ").append(send).append("], ");
      }

      go_commands.append("par: [[tello: [land]], [tello: [land]]]]");
      return go_commands;
    }

  public:
    static std::string generate(const std::string &cmds_generate)
    {
      if (cmds_generate == "rot_oscillate_1") {
        return generate_rot_oscillate_1(50, 1, 8);
      } else if (cmds_generate == "rot_oscillate_2") {
        return generate_rot_oscillate_2(50, 1, 8);
      }
      return std::string{};
    }
  };

  namespace base_machine
  {

    // ==============================================================================
    // Machine class
    // ==============================================================================

    class Machine : public TimerInterface
    {
#define SOURCE_CMDS_ERR "cmds_err"

      enum class States
      {
        starting = 0,
        ready,
        running,
        running_error,
        init_error,
      };

      States state_{States::starting};

      std::string source_{};

    public:
      explicit Machine(provoke::ProvokeNodeImpl &impl)
        : TimerInterface{"timer_pause", impl}
      {
      }

      ~Machine() override = default;

    private:
      void get_cmds(std::string &source, std::string &cmds)
      {
        // Test if we are supposed to calculate a command sequence.
        if (!impl_.cxt_.cmds_generate_.empty()) {
          source = impl_.cxt_.cmds_generate_;
          cmds = SequenceGenerator::generate(impl_.cxt_.cmds_generate_);
          CXT_MACRO_SET_PARAMETER(impl_.node_, impl_.cxt_, cmds_generate, "");
          return;
        }

        // test to see if the parameter cmds_go_ is non-negative. If it
        // is then we will load and execute the indicated cmds.
        if (impl_.cxt_.cmds_go_ < 0) {
          source.clear();
          cmds.clear();
          return;
        }

        // Pick one of the lists of commands to execute. Make a copy
        // of the string because it needs to stay around while it is being
        // executed. The parameter itself could change during execution.
        source = std::string{"cmds_"}.append(std::to_string(impl_.cxt_.cmds_go_));
        switch (impl_.cxt_.cmds_go_) {
          default:
          case 0:
            cmds = impl_.cxt_.cmds_0_;
            break;
          case 1:
            cmds = impl_.cxt_.cmds_1_;
            break;
          case 2:
            cmds = impl_.cxt_.cmds_2_;
            break;
          case 3:
            cmds = impl_.cxt_.cmds_3_;
            break;
          case 4:
            cmds = impl_.cxt_.cmds_4_;
            break;
        }

        // We have picked up this command, so reset the go parameter.
        CXT_MACRO_SET_PARAMETER(impl_.node_, impl_.cxt_, cmds_go, -1);

        if (cmds.empty()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Empty cmds for %s.",
                       source.c_str());
        }
      }

      Result get_yaml_args(const std::string &source, const std::string &cmds,
                           std::unique_ptr<YamlArgs> &yaml_args)
      {
        // Prepare to parse the command list.
        auto running_yaml_holder = std::make_shared<YamlHolder>();
        auto result = running_yaml_holder->from_string(cmds);

        // If the initial parse failed, report this and exit
        if (!result.succeeded()) {
          return Result::make_result(ResultCodes::parse_error,
                                     "Parse failed for %s with error: %s",
                                     source.c_str(), result.msg().c_str());
        }

        // Get a YamlArgs.
        result = running_yaml_holder->get_args(running_yaml_holder, yaml_args);

        // If the we couldn't get args, report this and exit
        if (!result.succeeded()) {
          return Result::make_result(ResultCodes::parse_error,
                                     "get_args failed for %s with error: %s",
                                     source.c_str(), result.msg().c_str());
        }

        return Result::success();
      }

      Result on_timer_starting(const rclcpp::Time &now)
      {
        (void) now;

        // if no error recovery commands, then we are ready.
        if (!impl_.cxt_.cmds_err_.empty()) {
          state_ = States::ready;
          return Result::success();
        }

        // Validate the error sequence. If the validation fails
        // move into error state and don't proceed.
        std::unique_ptr<YamlArgs> yaml_args;
        auto result = get_yaml_args(SOURCE_CMDS_ERR, impl_.cxt_.cmds_err_, yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "get_yaml_args() failed for "
                         SOURCE_CMDS_ERR
                         " with error: %s. Processing stopped.",
                       result.msg().c_str());
          state_ = States::init_error;
          return result;
        }

        // Validate the cmds in cmds_err.
        result = impl_.timer_dispatch_->validate_args(*yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "validate_args() failed for %s with error: %s. Processing stopped.",
                       SOURCE_CMDS_ERR, result.msg().c_str());
          state_ = States::init_error;
          return result;
        }

        // Validation succeeded so we can move to the ready state.
        state_ = States::ready;
        return Result::success();
      }

      Result on_timer_ready(const rclcpp::Time &now)
      {
        std::string cmds;

        // Get the commands
        get_cmds(source_, cmds);

        // If no string was returned, then no work to do.
        if (cmds.empty()) {
          return Result::success();
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Preparing to execute %s",
                    source_.c_str());
        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Cmds: %s",
                    cmds.c_str());

        // Validate the cmd sequence. First get the yaml_args.
        std::unique_ptr<YamlArgs> yaml_args;
        auto result = get_yaml_args(source_, cmds, yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "get_yaml_args() failed for %s with error: %s.",
                       source_.c_str(), result.msg().c_str());
          return result;
        }

        // Validate the cmds in source.
        result = impl_.timer_dispatch_->validate_args(*yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "validate_args() failed for %s with error: %s.",
                       source_.c_str(), result.msg().c_str());
          return result;
        }

        // Prepare for execution
        result = impl_.timer_dispatch_->prepare_from_args(now, *yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Prepare failed for %s with error: %s",
                       source_.c_str(), result.msg().c_str());
          return result;
        }

        // Transition to running state
        state_ = States::running;
        return Result::success();
      }

      Result on_timer_running(const rclcpp::Time &now, bool running_error_state)
      {
        // Dispatch the on_timer call.
        auto result = impl_.timer_dispatch_->on_timer(now);

        // If success, then just return.
        if (result.succeeded()) {
          return result;
        }

        // Try transitioning for the ready state
        state_ = States::ready;

        // Transition to the ready state if concluded
        if (result.concluded()) {
          RCLCPP_INFO(impl_.node_.get_logger(),
                      "on_timer() for %s concluded successfully",
                      source_.c_str());
          return result;
        }

        // An error occurred
        // If no cleanup commands, then just return the error and
        // transition to ready state.
        if (impl_.cxt_.cmds_err_.empty()) {
          RCLCPP_INFO(impl_.node_.get_logger(),
                      "on_timer() failed for %s with error: %s",
                      source_.c_str(), result.msg().c_str());
          return result;
        }

        // If we are currently executing
        // cleanup cmds, then give up.
        if (running_error_state) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "cleanup cmds for %s failed with error: %s",
                       source_.c_str(), result.msg().c_str());
          return result;
        }

        RCLCPP_ERROR(impl_.node_.get_logger(),
                    "on_timer() failed for %s with error: %s. Executing cleanup commands",
                    source_.c_str(), result.msg().c_str());

        // Try to execute the cleanup commands.
        // First get the yaml_args.
        std::unique_ptr<YamlArgs> yaml_args;
        result = get_yaml_args(source_, impl_.cxt_.cmds_err_, yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "get_yaml_args() for cleanup failed for %s with error: %s.",
                       SOURCE_CMDS_ERR, result.msg().c_str());
          return result;
        }

        // Prepare for execution of the cleanup commands
        result = impl_.timer_dispatch_->prepare_from_args(now, *yaml_args);
        if (!result.succeeded()) {
          RCLCPP_ERROR(impl_.node_.get_logger(),
                       "Prepare for cleanup failed for %s with error: %s",
                       SOURCE_CMDS_ERR, result.msg().c_str());
          return result;
        }

        // Transition to running_error state
        state_ = States::running_error;
        return Result::success();
      }

      Result on_timer(const rclcpp::Time &now) override
      {
        Result result{};
        switch (state_) {
          case States::starting:
            result = on_timer_starting(now);
            break;
          case States::ready:
            result = on_timer_ready(now);
            break;
          case States::running:
            result = on_timer_running(now, false);
            break;
          case States::running_error:
            result = on_timer_running(now, true);
            break;
          case States::init_error:
            result = Result::failure();
            break;
        }
        return result;
      }
    };

    std::unique_ptr<TimerInterface> factory(ProvokeNodeImpl &impl)
    {
      return std::unique_ptr<TimerInterface>{std::make_unique<Machine>(impl)};
    }
  }
}
