
#include "provoke_node_impl.hpp"

#include "provoke/provoke_node.hpp"
#include "sm_manager.hpp"

namespace provoke
{

  ProvokeNodeImpl::ProvokeNodeImpl(rclcpp::Node &node) :
    node_{node},
    sm_manager_{sm_manager_factory(*this)},
    sm_send_action_{sm_send_action_factory(*this)}
  {}

  ProvokeNodeImpl::~ProvokeNodeImpl()
  {}

  // ==============================================================================
  // ProvokeNode class
  // ==============================================================================


  class ProvokeNode : public rclcpp::Node
  {
    ProvokeNodeImpl impl_;

  public:
    ProvokeNode()
      : Node("provoke_node"), impl_(*this)
    {}
  };

  std::unique_ptr<rclcpp::Node> node_factory()
  {
    return std::unique_ptr<rclcpp::Node>(new ProvokeNode());
  }

}
