#ifndef PROVOKE_NODE_HPP
#define PROVOKE_NODE_HPP

#include "rclcpp/rclcpp.hpp"

namespace rclcpp
{
  class Node;
}
namespace provoke
{
  std::unique_ptr<rclcpp::Node> node_factory();
}

#endif //PROVOKE_NODE_HPP
