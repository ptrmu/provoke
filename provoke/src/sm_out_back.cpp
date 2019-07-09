
#include "sm_out_back.hpp"

namespace provoke
{
  namespace sm_out_back
  {
    Hub::Hub(Machine &machine) :
      machine_{machine}, gos_{
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_),
      sm_go_factory(machine.impl_)}
    {}

    SMResult Hub::set_running()
    {
      auto res = machine_.running_.prepare();
      if (!res.succeeded()) {
        return res;
      }
      return machine_.set_state(machine_.running_);
    }

    SMResult Hub::set_complete()
    {
      return machine_.set_state(machine_.complete_);
    }
  }

  std::unique_ptr<sm_out_back::Machine> sm_out_back_factory(provoke::ProvokeNodeImpl &impl)
  {
    return std::make_unique<sm_out_back::Machine>(impl);
  }
}