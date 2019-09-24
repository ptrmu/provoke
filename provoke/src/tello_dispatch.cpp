
#include "tello_dispatch.hpp"

namespace provoke
{
  TelloDispatch::TelloDispatch(ProvokeNodeImpl &impl, int inst_index) :
    SharedDispatch("tello_dispatch", impl, inst_index)
  {
    new_machine_ = [this](const std::string &cmd) -> std::unique_ptr<TelloInterface>
    {
      if (cmd.compare("takeoff") == 0) {
        return tello_machine_takeoff::factory(*this);
      } else if (cmd.compare("land") == 0) {
        return tello_machine_land::factory(*this);
      }
      return std::unique_ptr<TelloInterface>{};
    };
  }

  TelloDispatch::~TelloDispatch() = default;
}
