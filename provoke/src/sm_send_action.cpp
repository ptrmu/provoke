
#include "sm_send_action.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    void Hub::ready(int i) {
      machine_.ready_.prepare(i);
      machine_.set_state(machine_.ready_);
    }

    void Hub::waiting() {
      machine_.set_state(machine_.waiting_);
    }
  }

  std::unique_ptr<StateMachineInterface> sm_send_action_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::unique_ptr<StateMachineInterface>(new sm_send_action::Machine(impl));
  }
}