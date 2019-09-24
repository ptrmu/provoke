
#ifndef YAML_ARGS_HPP
#define YAML_ARGS_HPP

#include "result.hpp"
#include "yaml-cpp/yaml.h"

namespace provoke
{
  class ArgsInterface;

  class YamlHolder;

  class YamlSeq;

  class YamlArgs
  {
    std::shared_ptr<YamlHolder> holder_;  // keep reference to holder so pointers don't expire.
    YAML::Node yaml_node_;

  public:
    YamlArgs(std::shared_ptr<YamlHolder> &holder, YAML::Node yaml_node);

    Result get_seq(std::unique_ptr<YamlSeq> &seq);

    Result get_arg_str(const char *key, std::string &arg_str);
  };

  class YamlSeq
  {
    std::shared_ptr<YamlHolder> holder_;  // keep reference to holder so pointers don't expire.
    YAML::Node::const_iterator it_{};
    YAML::Node::const_iterator end_{};

    Result get_cmd_args(std::string &cmd, YAML::Node &args);

  public:
    YamlSeq(std::shared_ptr<YamlHolder> &holder, YAML::Node &yaml_node);

    bool done();

    bool next();

    Result get_cmd(std::string &cmd);

    Result get_args(std::unique_ptr<YamlArgs> &args);
  };

  class YamlHolder
  {
    YAML::Node yaml_node_{};

  public:
    YamlHolder() = default;

    Result from_string(const std::string &s);

    Result get_args(std::shared_ptr<YamlHolder> &holder, std::unique_ptr<YamlArgs> &args);
  };
}
#endif //YAML_ARGS_HPP
