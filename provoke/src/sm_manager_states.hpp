#ifndef SM_MANAGER_STATES_HPP
#define SM_MANAGER_STATES_HPP

#include <memory>

#include "state_machine_interface.hpp"

namespace provoke
{
  class ProvokeNodeImpl;
}

namespace sm_send_action
{
  class Machine;
}

namespace sm_manager
{
  class Machine;

  // ==============================================================================
  // StateA class
  // ==============================================================================

  class StateA : public provoke::StateInterface
  {
    provoke::ProvokeNodeImpl &impl_;
    Machine &machine_;

    std::unique_ptr<sm_send_action::Machine> sm_send_action_;

  public:
    StateA(provoke::ProvokeNodeImpl &impl, Machine &machine);

    ~StateA();

    virtual bool on_timer() override;
  };

  // ==============================================================================
  // States class
  // ==============================================================================

  class States
  {
  public:
    StateA state_a;

    States(provoke::ProvokeNodeImpl &impl, Machine &machine) :
      state_a(impl, machine)
    {}
  };
}
#endif //SM_MANAGER_STATES_HPP
