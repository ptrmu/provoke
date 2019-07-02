#ifndef SM_SEND_ACTION_STATES_HPP
#define SM_SEND_ACTION_STATES_HPP

#include "state_machine_interface.hpp"
#include "sm_manager.hpp"

namespace provoke
{
  class ProvokeNodeImpl;
}

namespace sm_send_action
{
  class Machine;

  // ==============================================================================
  // Ready state
  // ==============================================================================

  class Ready : public provoke::StateInterface
  {
    provoke::ProvokeNodeImpl &impl_;
    Machine &machine_;

  public:
    Ready(provoke::ProvokeNodeImpl &impl, Machine &machine);

    virtual bool on_timer() override;
  };

  // ==============================================================================
  // Waiting state
  // ==============================================================================

  class Waiting : public provoke::StateInterface
  {
    provoke::ProvokeNodeImpl &impl_;
    Machine &machine_;

  public:
    Waiting(provoke::ProvokeNodeImpl &impl, Machine &machine);

    virtual bool on_timer() override;
  };

  // ==============================================================================
  // States class
  // ==============================================================================

  class States
  {
  public:
    Ready ready_;
    Waiting waiting_;

    States(provoke::ProvokeNodeImpl &impl, Machine &machine) :
      ready_(impl, machine), waiting_(impl, machine)
    {}
  };
}
#endif //SM_SEND_ACTION_STATES_HPP
