
#ifndef YAML_ARGS_HPP
#define YAML_ARGS_HPP

#include "yaml-cpp/yaml.h"

#include "result.hpp"

namespace provoke
{
  class YamlArgs
  {
    YAML::Node yaml_node_{};

  public:
    YamlArgs() = default;

    Result from_string(const std::string &s)
    {
      std::stringstream ss{s};
      Result result{};

      try {
        auto yaml_node = YAML::Load(ss);
       }
      catch (YAML::ParserException &ex) {
       result = Result::make_result(ResultCodes::failure, "YAML parse error: '%s'", ex.what());
      }
      return result;
    }

  };
}
#endif //YAML_ARGS_HPP
