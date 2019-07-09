
#include "sm_out_back.hpp"

namespace provoke
{
  namespace sm_out_back
  {
    Hub::Hub(Machine &machine) :
      machine_{machine}, gos_{
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_)}
    {}

    SMResult Hub::sm_prepare(tf2::Vector3 velocity_mps, rclcpp::Duration go_duration,
                             rclcpp::Duration stop_duration, double msg_rate_hz)
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s (v:[%7.3f,%7.3f,%7.3f], go:%7.3f sec., stop:%7.3f sec., Hz:%7.3f)",
                  machine_.name_.c_str(),
                  velocity_mps.x(), velocity_mps.y(), velocity_mps.z(),
                  go_duration.seconds(), stop_duration.seconds(), msg_rate_hz);

      auto velocity_back = velocity_mps * -1;
      auto velocity_stop = tf2::Vector3{};

      gos_[0]->hub_.sm_prepare(velocity_mps, go_duration, msg_rate_hz);
      gos_[1]->hub_.sm_prepare(velocity_stop, stop_duration, msg_rate_hz);
      gos_[2]->hub_.sm_prepare(velocity_back, go_duration, msg_rate_hz);
      gos_[3]->hub_.sm_prepare(velocity_stop, stop_duration, msg_rate_hz);

      return set_running();
    }

    SMResult Hub::set_running()
    {
      auto res = machine_.running_.prepare();
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.running_);
    }

    SMResult Hub::set_complete()
    {
      return machine_.set_state(machine_.complete_);
    }

    SMResult Machine::_validate_args(const StateMachineArgs &args, tf2::Vector3 &velocity_mps,
                                     rclcpp::Duration &go_duration, rclcpp::Duration &stop_duration,
                                     double &msg_rate_hz)
    {
      double v_x = 0.0;
      double v_y = 0.0;
      double v_z = 0.0;
      go_duration = rclcpp::Duration{std::chrono::milliseconds{500}};
      stop_duration = rclcpp::Duration{std::chrono::milliseconds{250}};
      msg_rate_hz = 1.0;

      for (auto &arg: args) {
        auto &key = arg.first;
        auto value = arg.second.c_str();

        if (key == "v_x") {
          v_x = std::strtod(value, nullptr);

        } else if (key == "v_y") {
          v_y = std::strtod(value, nullptr);

        } else if (key == "v_z") {
          v_z = std::strtod(value, nullptr);

        } else if (key == "dur_out") {
          auto duration_secs = std::strtod(value, nullptr);
          go_duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(duration_secs * 1000)));

        } else if (key == "dur_back") {
          auto duration_secs = std::strtod(value, nullptr);
          stop_duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(duration_secs * 1000)));

        } else if (key == "hz") {
          msg_rate_hz = std::strtod(value, nullptr);

        }
      }

      velocity_mps = tf2::Vector3{v_x, v_y, v_z};

      return SMResult::success();
    }

    SMResult Machine::validate_args(const StateMachineArgs &args)
    {
      tf2::Vector3 velocity_mps;
      rclcpp::Duration go_duration{0, 0};
      rclcpp::Duration stop_duration{0, 0};
      double msg_rate_hz;

      return _validate_args(args, velocity_mps, go_duration, stop_duration, msg_rate_hz);
    }

    SMResult Machine::prepare_from_args(const StateMachineArgs &args)
    {
      tf2::Vector3 velocity_mps;
      rclcpp::Duration go_duration{0, 0};
      rclcpp::Duration stop_duration{0, 0};
      double msg_rate_hz;

      auto res = _validate_args(args, velocity_mps, go_duration, stop_duration, msg_rate_hz);
      if (!res.succeeded()) {
        return res;
      }
      return hub_.sm_prepare(velocity_mps, go_duration, stop_duration, msg_rate_hz);
    }
  }

  std::unique_ptr<sm_out_back::Machine> sm_out_back_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_out_back::Machine>(impl);
  }
}