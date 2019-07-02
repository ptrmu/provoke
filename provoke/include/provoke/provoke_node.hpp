#ifndef PROVOKE_NODE_HPP
#define PROVOKE_NODE_HPP

#include "rclcpp/rclcpp.hpp"


namespace provoke
{
  class ProvokeNodeImpl;

  class ProvokeNode : public rclcpp::Node
  {
    std::unique_ptr<ProvokeNodeImpl> impl_;

  public:
    ProvokeNode();

    ~ProvokeNode();
  };

}

#endif //PROVOKE_NODE_HPP
