
#include "provoke_node_impl.hpp"
#include "tello_dispatch.hpp"
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace timer_machine_tello
  {
    // This machine mostly just delegates to a TelloDispatch.
    class Machine : public ArgsInterface
    {
      TimerDispatch &dispatch_;

    public:
      explicit Machine(TimerDispatch &dispatch)
        : ArgsInterface{"timer_machine_tello", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine() override
      {
        dispatch_.tello_dispatch_.set_concluded();
      };

      Result on_timer(const rclcpp::Time &now) override
      {
        return dispatch_.tello_dispatch_.on_timer(now);
      }

      Result validate_args(YamlArgs &args) override
      {
        return dispatch_.tello_dispatch_.validate_args(args);
      }

      Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
      {
        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Prepare %s:%s",
                    dispatch_.name_.c_str(), name_.c_str());

        return dispatch_.tello_dispatch_.prepare_from_args(now, args);
      }
    };

    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch)
    {
      return std::unique_ptr<ArgsInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

