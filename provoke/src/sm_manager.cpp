
#include "sm_manager.hpp"

#include "sm_go.hpp"
#include "sm_out_back.hpp"
#include "sm_pause.hpp"
#include "sm_send_action.hpp"

namespace provoke
{
  namespace sm_manager
  {
    Hub::Hub(Machine &machine) :
      machine_{machine}
    {
      sm_land_ = sm_send_action_factory(machine_.impl_, "land");
      sm_takeoff_ = sm_send_action_factory(machine_.impl_, "takeoff");
      sm_go_ = sm_go_factory(machine_.impl_);
      sm_pause_ = sm_pause_factory(machine_.impl_);
      sm_out_back_ = sm_out_back_factory(machine_.impl_);

      sm_map.emplace("land", &*sm_land_);
      sm_map.emplace("takeoff", &*sm_takeoff_);
      sm_map.emplace("go", &*sm_go_);
      sm_map.emplace("pause", &*sm_pause_);
      sm_map.emplace("out_back", &*sm_out_back_);

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(machine_.impl_.node_, (*this), n, t, d)
      CXT_MACRO_INIT_PARAMETERS(SM_MANAGER_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED(machine_.impl_.node_, SM_MANAGER_ALL_PARAMS, validate_parameters)
    }

    Hub::~Hub() = default;

    void Hub::validate_parameters()
    {

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, machine_.impl_.node_.get_logger(), (*this), n, t, d)
      SM_MANAGER_ALL_PARAMS
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

  std::unique_ptr<sm_manager::Machine> sm_manager_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_manager::Machine>(impl);
  }

  void sm_prepare(sm_manager::Machine &machine, const std::string &poke_name)
  {
    machine.hub_.prepare(poke_name);
  }
}