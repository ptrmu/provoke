#ifndef PROVOKE_NODE_IMPL_HPP
#define PROVOKE_NODE_IMPL_HPP

#include <memory>

namespace rclcpp
{
  class Node;
}

namespace sm_manager
{
  class Machine;
}

namespace provoke
{
  class ProvokeNodeImpl
  {
  public:
    rclcpp::Node &node_;

  private:
    std::unique_ptr<sm_manager::Machine> sm_manager_;

  public:
    ProvokeNodeImpl(rclcpp::Node &node);

    ~ProvokeNodeImpl();
  };
}

#endif //PROVOKE_NODE_IMPL_HPP
