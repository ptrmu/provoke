
#include "yaml_args.hpp"

namespace provoke
{
  YamlArgs::YamlArgs(std::shared_ptr<YamlHolder> &holder, const YAML::Node &yaml_node) :
    holder_{holder}, yaml_node_{yaml_node}
  {}

  Result YamlArgs::get_seq(std::unique_ptr<YamlSeq> &seq)
  {
    seq.reset();

    // Make sure this node is a sequence
    if (!yaml_node_.IsSequence()) {
      return Result::make_result(ResultCodes::parse_error, "YAML node is not Sequence as expected.");
    }

    // Create a sequence object
    seq = std::make_unique<YamlSeq>(holder_, yaml_node_);
    return Result::success();
  }

  Result YamlArgs::get_arg_str(const char *key, std::string &arg_str)
  {
    arg_str.clear();
    if (*key) {
      if (yaml_node_.IsMap()) {
        auto value = yaml_node_[key];
        if (value.IsScalar()) {
          arg_str = value.Scalar();
        }
      }
    } else if (yaml_node_.IsScalar()) {
      arg_str = yaml_node_.Scalar();
    }

    return Result::success();
  }

  YamlSeq::YamlSeq(std::shared_ptr<YamlHolder> &holder, YAML::Node &yaml_node) :
    holder_{holder}
  {
    if (yaml_node.IsSequence()) {
      it_ = yaml_node.begin();
      end_ = yaml_node.end();
    }
  }

  bool YamlSeq::done()
  {
    return it_ == end_;
  }

  bool YamlSeq::next()
  {
    ++it_;
    return done();
  }

  Result YamlSeq::get_cmd_args(std::string &cmd, YAML::Node &args)
  {
    if (done()) {
      return Result::make_result(ResultCodes::logic_error, "Trying to get args from nothing");
    }

    auto &node = *it_;

    // Handle case: "land"
    if (node.IsScalar()) {
      cmd = node.Scalar();
      args = YAML::Node{};
      return Result::success();
    }

    // Handle case: "[x, y, z]"
    if (node.IsSequence()) {
      cmd.clear();
      args = YAML::Node{node};
      return Result::success();
    }

    // Handle case "pause: 3" and case "go: {duration:4}"
    if (node.IsMap()) {

      // There should only be one entry in the map.
      if (node.size() != 1) {
        return Result::make_result(ResultCodes::parse_error, "Incorrectly formed command.");
      }

      // The key should be a scalar
      const auto &pair = *node.begin();
      if (!pair.first.IsScalar()) {
        return Result::make_result(ResultCodes::parse_error, "Key must be string.");
      }

      cmd = pair.first.Scalar();
      args = pair.second;
      return Result::success();
    }

    return Result::make_result(ResultCodes::parse_error, "No Command Exists.");
  }

  Result YamlSeq::get_cmd(std::string &cmd)
  {
    cmd.clear();
    YAML::Node args;
    return get_cmd_args(cmd, args);
  }

  Result YamlSeq::get_args(std::unique_ptr<YamlArgs> &args)
  {
    args.reset();
    std::string cmd;
    YAML::Node args_node;
    auto result = get_cmd_args(cmd, args_node);
    if (!result.succeeded()) {
      return result;
    }

    args = std::make_unique<YamlArgs>(holder_, args_node);
    return Result::success();
  }

//  Result YamlArgs::parse_seq_begin(YAML::Node::const_iterator &it, YAML::Node::const_iterator &end)
//  {
//    // Check that we are positioned at a sequence
//    if (!yaml_node_.IsSequence()) {
//      return Result::make_result(ResultCodes::parse_error, "Parse error: expected sequence.");
//    }
//
//    // Return iterators stuff
//    it = yaml_node_.begin();
//    end = yaml_node_.end();
//    return Result::success();
//  }
//
//  Result YamlArgs::validate_cmd_seq(std::function<std::unique_ptr<ArgsInterface>(const std::string &)> get_machine)
//  {
//    YAML::Node::const_iterator it;
//    YAML::Node::const_iterator end;
//
//    // begin looking through the sequence
//    auto result = parse_seq_begin(it, end);
//    if (!result.succeeded()) {
//      return result;
//    }
//
//    while (it != end) {
//      auto &node = *it;
//
//      // For each element in the
//      std::string cmd;
//      YAML::Node *args;
//
//      // Handle case: "land"
//      if (node.IsScalar()) {
//        cmd = node.Scalar();
//        args = nullptr;
//      }
//
//        // handle case "pause: 3" and case "go: {duration:4}"
//      else if (node.IsMap()) {
//
//        // There should only be one entry in the map.
//        if (node.size() == 1) {
//          auto &pair = *poke.begin();
//
//          if (pair.first.IsScalar()) {
//            StateMachineInterface::StateMachineArgs sm_args{};
//            auto ret_val = from_poke_args(pair.second, sm_args);
//            if (ret_val) {
//              poke_name_list_.emplace_back(pair.first.Scalar());
//              poke_args_list_.emplace_back(sm_args);
//            }
//            return ret_val;
//          }
//          return yaml_error("yaml poke args name must be a scalar");
//        }
//        return yaml_error("yaml poke args must have only one name");
//      } else {
//        return Result::make_result(ResultCodes::parse_error, "Sequence entry must be Scalar or Map.");
//      }
//
//
//      ++it;
//    }
//
//    return result.concluded() ? Result::success() : result;
//
//  }
//
//  Result parse_seq_prepare(ArgsInterface &(*get_machine)(const char *cmd))
//  {
//    (void) get_machine;
//    return Result::failure();
//  }

  Result YamlHolder::from_string(const std::string &s)
  {
    std::stringstream ss{s};

    try {
      yaml_node_ = YAML::Load(ss);
    }
    catch (YAML::ParserException &ex) {
      return Result::make_result(ResultCodes::failure, "YAML parse error: '%s'", ex.what());
    }

    return Result::success();
  }

  Result YamlHolder::get_args(std::shared_ptr<YamlHolder> &holder, std::unique_ptr<YamlArgs> &args)
  {
    args = std::make_unique<YamlArgs>(holder, yaml_node_);
    return Result::success();
  }
}
