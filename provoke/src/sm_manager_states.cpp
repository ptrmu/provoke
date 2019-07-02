
#include "sm_manager_states.hpp"

#include "provoke_node_impl.hpp"
#include "sm_manager.hpp"
#include "sm_send_action.hpp"

namespace sm_manager
{

  StateA::StateA(provoke::ProvokeNodeImpl &impl, Machine &machine) :
    StateInterface(impl, "StateA"), impl_(impl), machine_(machine)
  {
    sm_send_action_ = std::make_unique<sm_send_action::Machine>(impl_);
  }

  StateA::~StateA() = default;

  bool StateA::on_timer()
  {
    machine_.set_state(machine_.states_->state_a);
    return false;
  }
}
