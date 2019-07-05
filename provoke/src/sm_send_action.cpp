
#include "sm_send_action.hpp"

namespace provoke
{
  namespace sm_send_action
  {
    void Hub::set_ready(int i)
    {
      machine_.ready_.prepare(i);
      machine_.set_state(machine_.ready_);
    }

    void Hub::set_waiting()
    {
      machine_.set_state(machine_.waiting_);
    }
  }

  std::unique_ptr<sm_send_action::Machine>
  sm_send_action_factory(provoke::ProvokeNodeImpl &impl, const std::string &action)
  {
    return std::make_unique<sm_send_action::Machine>(impl, action);
  }
}