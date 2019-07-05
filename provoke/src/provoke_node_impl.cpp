
#include "provoke_node_impl.hpp"

#include "provoke/provoke_node.hpp"
#include "sm_manager.hpp"

namespace provoke
{

  // ==============================================================================
  // ProvokeNodeImpl class
  // ==============================================================================

  ProvokeNodeImpl::ProvokeNodeImpl(rclcpp::Node &node) :
    node_{node}, sm_manager_{sm_manager_factory(*this)}
  {
    sm_prepare(*sm_manager_, "out_back");

    timer_ = node_.create_wall_timer(
      std::chrono::milliseconds{timer_interval_ms},
      [this]() -> void
      {
        sm_manager_->state().on_timer(node_.now());
      });
  }

  ProvokeNodeImpl::~ProvokeNodeImpl()
  {}

  // ==============================================================================
  // ProvokeNode class
  // ==============================================================================

  class ProvokeNode : public rclcpp::Node
  {
    ProvokeNodeImpl impl_;

  public:
    ProvokeNode(rclcpp::NodeOptions &options)
      : Node("provoke_node", options), impl_(*this)
    {
      RCLCPP_INFO(get_logger(), "provoke_node ready");
    }
  };

  std::shared_ptr<rclcpp::Node> node_factory(rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new ProvokeNode(options));
  }

}
