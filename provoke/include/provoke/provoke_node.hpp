#ifndef PROVOKE_NODE_HPP
#define PROVOKE_NODE_HPP

#include "rclcpp/rclcpp.hpp"

namespace provoke
{
  std::shared_ptr<rclcpp::Node> node_factory(rclcpp::NodeOptions &options);
}

#endif //PROVOKE_NODE_HPP
