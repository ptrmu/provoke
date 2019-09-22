
#include "provoke_node_impl.hpp"


#include "base_interface.hpp"
#include "provoke/provoke_node.hpp"
#include "sm_manager.hpp"
#include "timer_dispatch.hpp"

namespace provoke
{

  // ==============================================================================
  // ProvokeNodeImpl class
  // ==============================================================================

  ProvokeNodeImpl::ProvokeNodeImpl(rclcpp::Node &node) :
    node_{node},
    timer_dispatch_{std::make_unique<TimerDispatch>(*this)},
    sm_manager_{sm_manager_factory(*this)},
    base_machine_{base_machine::factory(*this)}
  {
    sm_manager_->hub_.sm_prepare();

    timer_ = node_.create_wall_timer(
      std::chrono::milliseconds{timer_interval_ms},
      [this]() -> void
      {
        sm_manager_->state().on_timer(node_.now());
        base_machine_->on_timer(node_.now());
      });
  }

  ProvokeNodeImpl::~ProvokeNodeImpl() = default;

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
