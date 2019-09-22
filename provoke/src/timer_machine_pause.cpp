
#include "timer_interface.hpp"

namespace provoke
{
  namespace timer_machine_pause
  {
    enum class States
    {
      ready = 0,
      waiting,
      timeout,
      logic_error,
      failure,
    };

    class Machine : public TimerInterface
    {
      BaseInterface &dispatch_;

    public:
      explicit Machine(BaseInterface &dispatch)
        : TimerInterface{"timer_machine_pause", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine() override = default;

      Result on_timer(rclcpp::Time now) override
      {
        (void) now;
        return Result::failure();
      }

      Result validate_args(YamlArgs &args) override
      {
        (void) args;
        return Result::failure();
      }

      Result prepare_from_args(YamlArgs &args) override
      {
        (void) args;
        return Result::failure();
      }
    };

    std::unique_ptr<TimerInterface> factory(BaseInterface &dispatch)
    {
      return std::unique_ptr<TimerInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

