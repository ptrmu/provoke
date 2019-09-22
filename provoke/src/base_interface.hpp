
#ifndef BASE_INTERFACE_HPP
#define BASE_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"

#include "result.hpp"

namespace provoke
{
  class ProvokeNodeImpl;

  class BaseInterface
  {
  public:
    const std::string name_;
    ProvokeNodeImpl &impl_;

    BaseInterface(std::string name, ProvokeNodeImpl &impl) :
      name_{std::move(name)}, impl_{impl}
    {}

    BaseInterface() = delete;

    virtual ~BaseInterface() = default;

    virtual Result on_timer(rclcpp::Time now);
  };

  namespace base_machine
  {
    std::unique_ptr<BaseInterface> factory(ProvokeNodeImpl &impl);
  }
}
#endif //BASE_INTERFACE_HPP
