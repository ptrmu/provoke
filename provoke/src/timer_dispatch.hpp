
#ifndef TIMER_DISPATCH_HPP
#define TIMER_DISPATCH_HPP

#include "args_interface.hpp"
#include "shared_dispatch.hpp"

namespace provoke
{
  class TelloDispatch;

  class TimerDispatch : public SharedDispatch<ArgsInterface>
  {
  public:
    TelloDispatch &tello_dispatch_;

    TimerDispatch(ProvokeNodeImpl &impl, int inst_index, TelloDispatch &tello_dispatch);

    ~TimerDispatch() override;
  };

  namespace timer_machine_pause
  {
    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch);
  }

  namespace timer_machine_par
  {
    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch);
  }

  namespace timer_machine_tello
  {
    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch);
  }
}
#endif //TIMER_DISPATCH_HPP
