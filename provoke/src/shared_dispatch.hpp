
#ifndef SHARED_DISPATCH_HPP
#define SHARED_DISPATCH_HPP

#include "args_interface.hpp"
#include "provoke_node_impl.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  template<class TMachineInterface>
  class SharedDispatch : public ArgsInterface
  {
    enum class States
    {
      concluded = 0,
      running,
    };

    using NewMachineFunc = std::function<std::unique_ptr<TMachineInterface>(const std::string &)>;

    States state_{States::concluded};

    std::unique_ptr<YamlSeq> running_seq_{};

    Result prepare_cmd_from_args(const rclcpp::Time &now)
    {
      // If we are at the end of the sequence, then return conclusion
      if (running_seq_->done()) {
        return Result::conclusion();
      }

      // Get the command from this item in the sequence.
      std::string cmd;
      auto result = running_seq_->get_cmd(cmd);

      // If we couldn't get a command return the error.
      if (!result.succeeded()) {
        return result;
      }

      // An empty command also is an error.
      if (cmd.empty()) {
        return Result::make_result(ResultCodes::parse_error, "Command is empty. (prepare)");
      }

      // get a machine for this command
      running_machine_ = new_machine_(cmd);

      // If we couldn't get a machine, then the name is bad
      if (running_machine_.get() == nullptr) {
        return Result::make_result(ResultCodes::parse_error, "Machine %s could not be found. (prepare)", cmd.c_str());
      }

      // Get the args for this cmd
      std::unique_ptr<YamlArgs> cmd_args;
      result = running_seq_->get_args(cmd_args);

      // Return any error
      if (!result.succeeded()) {
        return result;
      }

      // Ask the machine to validate the args
      result = running_machine_->prepare_from_args(now, *cmd_args);

      // Return any error
      if (!result.succeeded()) {
        return result;
      }

      // Move to the next command in the sequence.
      running_seq_->next();
      return Result::success();
    }

    Result prepare_cmd_from_seq(const rclcpp::Time &now)
    {
      // must be in running state
      assert(state_ == States::running);

      // Prepare the machine that the seq points to. Also advance the seq.
      auto result = prepare_cmd_from_args(now);

      // If there was an error, move to concluded state
      if (!result.succeeded()) {
        set_concluded();
      }

      return result;
    }

    Result on_timer_concluded(const rclcpp::Time &now)
    {
      (void) now;
      return Result::conclusion();
    }

    Result on_timer_running(const rclcpp::Time &now)
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
      return prepare_cmd_from_seq(now);
    }

  protected:
    NewMachineFunc new_machine_{};
    std::unique_ptr<TMachineInterface> running_machine_{};

  public:
    SharedDispatch(std::string name, ProvokeNodeImpl &impl, int inst_index) :
      ArgsInterface(name.append("_").append(std::to_string(inst_index)), impl)
    {}

    ~SharedDispatch() override = default;

    void set_concluded()
    {
      if (running_seq_) {
        running_seq_.reset();
      }
      if (running_machine_) {
        running_machine_.reset();
      }
      state_ = States::concluded;
    }

    Result on_timer(const rclcpp::Time &now) override
    {
      Result result{};
      switch (state_) {
        case States::concluded:
          result = on_timer_concluded(now);
          break;
        case States::running:
          result = on_timer_running(now);
          break;
      }
      return result;
    }

    Result validate_args(YamlArgs &args) override
    {
      // Get a sequence from these args.
      std::unique_ptr<YamlSeq> seq;
      auto result = args.get_seq(seq);

      // If we couldn't get a sequence return the error.
      if (!result.succeeded()) {
        return result;
      }

      while (!seq->done()) {
        std::string cmd;

        result = seq->get_cmd(cmd);

        // If we couldn't get a command return the error.
        if (!result.succeeded()) {
          return result;
        }

        // An empty command also is an error.
        if (cmd.empty()) {
          return Result::make_result(ResultCodes::parse_error, "Command is empty.");
        }

        // get a machine for this command
        auto machine = new_machine_(cmd);

        // If we couldn't get a machine, then the name is bad
        if (machine == nullptr) {
          return Result::make_result(ResultCodes::parse_error, "Machine %s could not be found.", cmd.c_str());
        }

        // Get the args for this cmd
        std::unique_ptr<YamlArgs> cmd_args;
        result = seq->get_args(cmd_args);

        // Return any error
        if (!result.succeeded()) {
          return result;
        }

        // Ask the machine to validate the args
        result = machine->validate_args(*cmd_args);

        // Return any error
        if (!result.succeeded()) {
          return result;
        }

        // Move to the next command in the sequence.
        seq->next();
      }

      return Result::success();
    }

    Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
    {
      // Get the sequence from the args
      auto result = args.get_seq(running_seq_);
      if (!result.succeeded()) {
        RCLCPP_ERROR(impl_.node_.get_logger(),
                     "%s expects a sequence as an argument",
                     name_);
        return result;
      }

      state_ = States::running;

      // Prepare this command
      return prepare_cmd_from_seq(now);
    }
  };
}
#endif //SHARED_DISPATCH_HPP
