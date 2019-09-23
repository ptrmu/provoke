
#include "timer_dispatch.hpp"

#include "provoke_node_impl.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  TimerDispatch::TimerDispatch(ProvokeNodeImpl &impl, int group_index) :
    TimerInterface("TimerDispatch", impl),
    running_seq_{},
    running_machine_{},
    group_index_{group_index}
  {
  }

  TimerDispatch::~TimerDispatch() = default;


  std::unique_ptr<TimerInterface> TimerDispatch::new_machine(const std::string &cmd)
  {
    if (cmd.compare("pause") == 0) {
      return timer_machine_pause::factory(*this);
    } else if (cmd.compare("par") == 0) {
      return timer_machine_par::factory(*this);
    }
    return std::unique_ptr<TimerInterface>{};
  }

  void TimerDispatch::set_concluded()
  {
    if (running_seq_) {
      running_seq_.release();
    }
    if (running_machine_) {
      running_machine_.release();
    }
    state_ = States::concluded;
  }

  Result TimerDispatch::prepare_cmd_from_seq()
  {
    // must be in running state
    assert(state_ == States::running);

    // Prepare the machine that the seq points to. Also advance the seq.
    auto result = prepare_cmd_from_args(*running_seq_, [this](const std::string &cmd) -> TimerInterface *
    {
      running_machine_ = new_machine(cmd);
      return running_machine_.get();
    });

    // If there was an error, move to concluded state
    if (!result.succeeded()) {
      set_concluded();
    }

    return result;
  }

  Result TimerDispatch::on_timer_concluded(rclcpp::Time now)
  {
    (void) now;
    return Result::conclusion();
  }

  Result TimerDispatch::on_timer_running(rclcpp::Time now)
  {
    assert(running_machine_);

    // Dispatch to the running machine
    auto result = running_machine_->on_timer(now);

    // If no problem, then just return
    if (result.succeeded()) {
      return result;
    }

    // If there was an error, then clean up and return the error
    if (!result.concluded()) {
      set_concluded();
      return result;
    }

    // Prepare the next machine in the argument sequence.
    return prepare_cmd_from_seq();
  }

  Result TimerDispatch::on_timer(rclcpp::Time now)
  {
    switch (state_) {
      case States::concluded:
        return on_timer_concluded(now);
      case States::running:
        return on_timer_running(now);
      default:
        state_ = States::concluded;
        return Result::conclusion();
    }
  }

  Result TimerDispatch::validate_args(YamlArgs &args)
  {
    return validate_cmd_seq(args, [this](const std::string &cmd) -> std::unique_ptr<TimerInterface>
    {
      return new_machine(cmd);
    });
  }

  Result TimerDispatch::prepare_from_args(YamlArgs &args)
  {
    // Get the sequence from the args
    auto result = args.get_seq(running_seq_);
    if (!result.succeeded()) {
      RCLCPP_ERROR(impl_.node_.get_logger(),
                   "Timer dispatch (group_index: %1) expects a sequence as an argument",
                   group_index_);
      return result;
    }

    state_ = States::running;

    // Prepare this command
    return prepare_cmd_from_seq();
  }
}
