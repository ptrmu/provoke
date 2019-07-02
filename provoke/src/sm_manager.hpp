#ifndef SM_MANAGER_HPP
#define SM_MANAGER_HPP

#include <memory>

#include "state_machine_interface.hpp"

namespace provoke
{
  class ProvokeNodeImpl;
}

namespace sm_manager
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

#endif //SM_MANAGER_HPP
