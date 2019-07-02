
#include "sm_send_action.hpp"

#include "sm_send_action_states.hpp"

namespace sm_send_action
{
  Machine::Machine(provoke::ProvokeNodeImpl &impl)
    : states_(std::make_unique<States>(impl, *this))
  {

  }

  Machine::~Machine()
  {}
}