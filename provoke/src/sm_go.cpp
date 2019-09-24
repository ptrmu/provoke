
#include "sm_go.hpp"

namespace provoke
{
  namespace sm_go
  {
    Hub::Hub(Machine &machine) :
      machine_{machine}
    {
      cmd_vel_pub_ = machine_.impl_.node_.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }

    SMResult Hub::set_ready()
    {
      return machine_.set_state(machine_.ready_);
    }

    SMResult Hub::sm_prepare(tf2::Vector3 velocity_mps, rclcpp::Duration duration, double msg_rate_hz)
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                  "Prepare sm:%s (v:[%7.3f,%7.3f,%7.3f], duration:%7.3f sec., Hz:%7.3f)",
                  machine_.name_.c_str(),
                  velocity_mps.x(), velocity_mps.y(), velocity_mps.z(),
                  duration.seconds(), msg_rate_hz);

      velocity_mps_ = velocity_mps;
      auto res = machine_.ready_.prepare(duration, msg_rate_hz);
      if (!res.succeeded()) {
        return res;
      }
      return set_ready();
    }

    SMResult Hub::set_waiting(const rclcpp::Time end_time, rclcpp::Time next_msg_time, rclcpp::Duration inter_msg_duration)
    {
      auto res = machine_.waiting_.prepare(end_time, next_msg_time, inter_msg_duration);
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.waiting_);
    }

    static double clamp(const double v, const double min, const double max)
    {
      return v > max ? max : (v < min ? min : v);
    }

    void Hub::send_go()
    {
      RCLCPP_INFO(machine_.impl_.node_.get_logger(), "Send Go: x:%7.3f, y:%7.3f, z:%7.3f",
                  velocity_mps_.x(), velocity_mps_.y(), velocity_mps_.z());

      geometry_msgs::msg::Twist twist;
      twist.linear.x = clamp(velocity_mps_.x(), -1.0, 1.0);
      twist.linear.y = clamp(velocity_mps_.y(), -1.0, 1.0);
      twist.linear.z = clamp(velocity_mps_.z(), -1.0, 1.0);
      twist.angular.z = clamp(0., -1.0, 1.0);
      cmd_vel_pub_->publish(twist);
    }

    SMResult Machine::_validate_args(const StateMachineArgs &args, tf2::Vector3 &velocity_mps,
                                     rclcpp::Duration &duration, double &msg_rate_hz)
    {
      double v_x = 0.0;
      double v_y = 0.0;
      double v_z = 0.0;
      duration = rclcpp::Duration{std::chrono::seconds{1}};
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

        } else if (key == "duration") {
          auto duration_secs = std::strtod(value, nullptr);
          duration = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(duration_secs * 1000)));

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
      rclcpp::Duration duration{0, 0};
      double msg_rate_hz;

      return _validate_args(args, velocity_mps, duration, msg_rate_hz);
    }

    SMResult Machine::prepare_from_args(const StateMachineArgs &args)
    {
      tf2::Vector3 velocity_mps;
      rclcpp::Duration duration{0, 0};
      double msg_rate_hz;

      auto res = _validate_args(args, velocity_mps, duration, msg_rate_hz);
      if (!res.succeeded()) {
        return res;
      }
      return hub_.sm_prepare(velocity_mps, duration, msg_rate_hz);
    }
  }

  std::unique_ptr<sm_go::Machine> sm_go_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_go::Machine>(impl);
  }
}