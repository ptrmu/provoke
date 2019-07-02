
#include "sm_manager.hpp"

#include "sm_manager_states.hpp"

namespace sm_manager
{
  Machine::Machine(provoke::ProvokeNodeImpl &impl)
    : states_(std::make_unique<States>(impl, *this))
  {
  }

  Machine::~Machine()
  {}
}