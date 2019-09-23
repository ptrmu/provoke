
#include "timer_interface.hpp"

#include "yaml_args.hpp"

namespace provoke
{
  Result TimerInterface::validate_args(YamlArgs &args)
  {
    (void) args;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'validate_args()'",
                               name_.c_str());
  }

  Result TimerInterface::prepare_from_args(YamlArgs &args)
  {
    (void) args;
    return Result::make_result(ResultCodes::logic_error,
                               "Machine:%s has not implemented 'prepare_from_args()'",
                               name_.c_str());
  }

  Result TimerInterface::validate_cmd_seq(YamlArgs &args,
                                          std::function<std::unique_ptr<TimerInterface>(
                                            const std::string &)> get_machine)
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
      std::unique_ptr<YamlArgs> args;
      result = seq->get_args(args);

      // If we couldn't get args return the error.
      if (!result.succeeded()) {
        return result;
      }

      auto machine = get_machine(cmd);

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
      result = machine->validate_args(*args);

      // Return any error
      if (!result.succeeded()) {
        return result;
      }

      // Move to the next command in the sequence.
      seq->next();
    }

    return Result::success();

  }

}

