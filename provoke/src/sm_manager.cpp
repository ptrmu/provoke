
#include "sm_manager.hpp"

namespace provoke
{
  namespace sm_manager
  {
    void PrepareAndSet::state_a() {
      machine_.set_state(machine_.state_a_);
    }
  }

  std::unique_ptr<StateMachineInterface> sm_manager_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::unique_ptr<StateMachineInterface>(new sm_manager::Machine(impl));
  }
}