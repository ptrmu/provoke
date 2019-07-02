
#include "provoke_node_impl.hpp"

#include "provoke/provoke_node.hpp"
#include "sm_manager.hpp"

namespace provoke
{

  ProvokeNodeImpl::ProvokeNodeImpl(rclcpp::Node &node) :
    node_{node}, sm_manager_{std::make_unique<sm_manager::Machine>(*this)}
  {}

  ProvokeNodeImpl::~ProvokeNodeImpl()
  {}
}
