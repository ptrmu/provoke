#ifndef SM_SEND_ACTION_HPP
#define SM_SEND_ACTION_HPP

#include <memory>

#include "state_machine_interface.hpp"

namespace provoke
{
  class ProvokeNodeImpl;
}

namespace sm_send_action
{
  class States;

  class Machine : public provoke::StateMachineInterface
  {
  public:
    std::unique_ptr<States> states_;

    Machine(provoke::ProvokeNodeImpl &impl);

    ~Machine();
  };
}

#endif //SM_SEND_ACTION_HPP
