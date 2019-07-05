
#include "sm_manager.hpp"

#include "sm_out_back.hpp"

namespace provoke
{
  namespace sm_manager
  {
    Hub::Hub(Machine &machine) :
      machine_{machine}, sm_out_back_{sm_out_back_factory(machine_.impl_)}
    {}

    Hub::~Hub() = default;

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