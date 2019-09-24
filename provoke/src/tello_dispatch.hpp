
#ifndef TELLO_DISPATCH_HPP
#define TELLO_DISPATCH_HPP

#include "shared_dispatch.hpp"
#include "tello_interface.hpp"

namespace provoke
{

  class TelloDispatch : public SharedDispatch<TelloInterface>
  {
  public:
    TelloDispatch(ProvokeNodeImpl &impl, int inst_index);

    ~TelloDispatch() override;
  };

  namespace tello_machine_takeoff
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch);
  }

  namespace tello_machine_land
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch);
  }
}
#endif //TELLO_DISPATCH_HPP
