
#include "provoke_node_impl.hpp"
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace timer_machine_par
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
      TimerDispatch &dispatch_;

    public:
      explicit Machine(TimerDispatch &dispatch)
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
        // Par takes a sequence of sequences as the argument
        std::unique_ptr<YamlSeq> seq;
        auto result = args.get_seq(seq);
        if (!result.succeeded()) {
          return Result::make_result(ResultCodes::parse_error, "par machine requires a sequence argument.");
        }

        // Loop over the elements in the sequence
        while (!seq->done()) {
          // Get the args for this item.
          std::unique_ptr<YamlArgs> dispatch_args;
          result = seq->get_args(dispatch_args);
          if (!result.succeeded()) {
            return Result::make_result(ResultCodes::parse_error, "par machine can not get an argument.");
          }

          // See if the dispatcher validates the list.
          result = impl_.timer_dispatchs_[0]->validate_args(*dispatch_args);
          if (!result.succeeded()) {
            return result;
          }

          // Move to the next element in the sequence
          seq->next();
        }

        return Result::success();
      }

      Result prepare_from_args(YamlArgs &args) override
      {
        (void) args;
        return Result::failure();
      }
    };

    std::unique_ptr<TimerInterface> factory(TimerDispatch &dispatch)
    {
      return std::unique_ptr<TimerInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

