
#ifndef DISPATCH_INTERFACE_HPP
#define DISPATCH_INTERFACE_HPP

namespace provoke
{
  class ProvokeNodeImpl;

  class DispatchInterface
  {
  public:
    ProvokeNodeImpl &impl_;

    DispatchInterface(ProvokeNodeImpl &impl) :
      impl_{impl}
    {}
  };
}
#endif //DISPATCH_INTERFACE_HPP
