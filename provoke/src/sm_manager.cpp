
#include "sm_manager.hpp"

#include "sm_go.hpp"
#include "sm_out_back.hpp"
#include "sm_pause.hpp"
#include "sm_send_action.hpp"
#include "yaml-cpp/yaml.h"

namespace provoke
{
  namespace sm_manager
  {

    // ==============================================================================
    // YamlParser class
    // ==============================================================================

    class YamlParser
    {
      YAML::Node yaml_node_{};

      bool yaml_error(const std::string &s)
      {
        error_msg_ = "YamlParser error -";
        error_msg_.append(s);
        return false;
      }

      bool from_poke_args(const YAML::Node &poke, StateMachineInterface::StateMachineArgs &sm_args)
      {
        // handle the "xx: 3" case
        if (poke.IsScalar()) {
          sm_args.emplace("", poke.Scalar());
          return true;
        }

        // handle the "xx: {a:32}" case
        if (poke.IsMap()) {
          if (poke.size() > 0) {

            // loop over all the elements in the map and add them to the args map.
            for (auto iter = poke.begin(); iter != poke.end(); ++iter) {
              auto &pair = *iter;
              if (pair.first.IsScalar() && pair.second.IsScalar()) {

                sm_args.emplace(pair.first.Scalar(), pair.second.Scalar());
                return true;

              } else {
                return yaml_error("key and value must be scalars.");
              }
            }
          } else {
            return yaml_error("need an argument in the map");
          }
        }

        return yaml_error("mal formed arguments. Should be 'xx: 3' or xx: { a:34 }'");
      }

      bool from_poke(const YAML::Node &poke)
      {
        // handle case "land"
        if (poke.IsScalar()) {
          poke_name_list_.emplace_back(poke.Scalar());
          poke_args_list_.emplace_back(StateMachineInterface::StateMachineArgs{});
          return true;
        }

        // handle case "pause: 3" and case "go: {duration:4}"
        if (poke.IsMap()) {

          // There should only be one entry in the map.
          if (poke.size() == 1) {
            auto &pair = *poke.begin();

            if (pair.first.IsScalar()) {
              StateMachineInterface::StateMachineArgs sm_args{};
              auto ret_val = from_poke_args(pair.second, sm_args);
              if (ret_val) {
                poke_name_list_.emplace_back(pair.first.Scalar());
                poke_args_list_.emplace_back(sm_args);
              }
              return ret_val;
            }
            return yaml_error("yaml poke args name must be a scalar");
          }
          return yaml_error("yaml poke args must have only one name");
        }
        return yaml_error("yaml poke args must be Scalar or Map");
      }

      bool from_poke_sequence(const YAML::Node &poke_list)
      {
        for (auto iter = poke_list.begin(); iter != poke_list.end(); ++iter) {
          if (!from_poke(*iter)) {
            return false;
          }
        }
        return true;
      }

      bool from_poke_list(const YAML::Node &poke_list)
      {
        // handle case "land" and case "pause: 3"
        if (poke_list.IsScalar() || poke_list.IsMap()) {
          return from_poke(poke_list);
        }

        // handle case with a sequence
        if (poke_list.IsSequence()) {
          return from_poke_sequence(poke_list);
        }

        return yaml_error("yaml args must be Scalar, Map, or Sequence.");
      }

      bool from_stream(std::istream &in)
      {
        error_msg_.clear();
        poke_name_list_.clear();
        poke_args_list_.clear();

        try {
          auto yaml_node = YAML::Load(in);
          return from_poke_list(yaml_node);
        }
        catch (YAML::ParserException &ex) {
          error_msg_ = ex.what();
        }
        return false;
      }

    public:
      std::string error_msg_{};
      std::vector<std::string> poke_name_list_;
      std::vector<StateMachineInterface::StateMachineArgs> poke_args_list_;

      bool from_string(const std::string &s)
      {
        std::stringstream ss{s};
        return from_stream(ss);
      }
    };

    // ==============================================================================
    // Hub class
    // ==============================================================================

    Hub::Hub(Machine &machine) :
      machine_{machine}
    {
      sm_land_ = sm_send_action_factory(machine_.impl_, "land");
      sm_takeoff_ = sm_send_action_factory(machine_.impl_, "takeoff");
      sm_go_ = sm_go_factory(machine_.impl_);
      sm_pause_ = sm_pause_factory(machine_.impl_);
      sm_out_back_ = sm_out_back_factory(machine_.impl_);

      sm_map_.emplace("land", &*sm_land_);
      sm_map_.emplace("takeoff", &*sm_takeoff_);
      sm_map_.emplace("go", &*sm_go_);
      sm_map_.emplace("pause", &*sm_pause_);
      sm_map_.emplace("out_back", &*sm_out_back_);

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(machine_.impl_.node_, (*this), n, t, d)
      CXT_MACRO_INIT_PARAMETERS(SM_MANAGER_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED(machine_.impl_.node_, SM_MANAGER_ALL_PARAMS, validate_parameters)
    }

    Hub::~Hub() = default;

    bool Hub::validate_sm_args(YamlParser &yaml_parser, int poke_list_idx)
    {
      for (size_t i = 0; i < yaml_parser.poke_name_list_.size(); i += 1) {

        auto &poke_name = yaml_parser.poke_name_list_[i];
        auto marker_pair = sm_map_.find(poke_name);

        if (marker_pair == sm_map_.end()) {
          RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                      "YAML parse of poke_list_%d failed: poke with name %s does not exist (poke #%d).",
                      poke_list_idx, poke_name.c_str(), i + 1);
          return false;
        }

        auto validate_results = marker_pair->second->validate_args(yaml_parser.poke_args_list_[i]);
        if (!validate_results.empty()) {
          RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                      "YAML parse of poke_list_%d failed: poke %s (poke #%d) validate_args failed with error: '%s'.",
                      poke_list_idx, poke_name.c_str(), i + 1, validate_results.c_str());
          return false;
        }
      }

      return true;
    }

    void Hub::validate_parameters()
    {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, machine_.impl_.node_.get_logger(), (*this), n, t, d)
      SM_MANAGER_ALL_PARAMS

      // Validate the poke_list_n_ parameters
      YamlParser yaml_parser;
      for (size_t i = 0; i < poke_lists_.size(); i += 1) {
        poke_list_valids_[i] = false;
        if (!yaml_parser.from_string(*poke_lists_[i])) {
          RCLCPP_ERROR(machine_.impl_.node_.get_logger(),
                       "YAML parse of poke_list_%d failed with error: '%s'",
                       i + 1, yaml_parser.error_msg_.c_str());

        } else if (validate_sm_args(yaml_parser, i + 1)) {
          poke_list_valids_[i] = true;
          RCLCPP_INFO(machine_.impl_.node_.get_logger(),
                      "YAML parse of poke_list_%d succeeded", i + 1);
        }
      }
    }

    void Hub::prepare(const std::string &poke_name)
    {
      if (poke_name == "out_back") {
        sm_prepare(*sm_out_back_,
                   tf2::Vector3{0.0, 1.0, 0.0},
                   std::chrono::milliseconds{3000},
                   std::chrono::milliseconds{500},
                   0.0);

        set_state_a(&*sm_out_back_);

      } else {
        set_complete();
      }
    }

    void Hub::set_state_a(StateMachineInterface *sub_machine)
    {
      machine_.state_a_.prepare(sub_machine);
      machine_.set_state(machine_.state_a_);
    }

    void Hub::set_complete()
    {
      machine_.complete_.prepare();
      machine_.set_state(machine_.complete_);
    }
  }

// ==============================================================================
// sm_manager::Machine
// ==============================================================================

  std::unique_ptr<sm_manager::Machine> sm_manager_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_manager::Machine>(impl);
  }

  void sm_prepare(sm_manager::Machine &machine, const std::string &poke_name)
  {
    machine.hub_.prepare(poke_name);
  }

}