
#include "sm_send_action_states.hpp"

#include "provoke_node_impl.hpp"
#include "sm_send_action.hpp"

namespace sm_send_action
{

  Ready::Ready(provoke::ProvokeNodeImpl &impl, Machine &machine) :
    StateInterface(impl, "StateA"), impl_(impl), machine_(machine)
  {
  }

  bool Ready::on_timer()
  {
    machine_.set_state(machine_.states_->waiting_);
    return false;
  }

  Waiting::Waiting(provoke::ProvokeNodeImpl &impl, Machine &machine) :
    StateInterface(impl, "StateA"), impl_(impl), machine_(machine)
  {
  }

  bool Waiting::on_timer()
  {
    machine_.set_state(machine_.states_->waiting_);
    return false;
  }
}
