
#include "provoke_node_impl.hpp"


#include "timer_interface.hpp"
#include "provoke/provoke_node.hpp"
#include "tello_dispatch.hpp"
#include "timer_dispatch.hpp"

namespace provoke
{

  // ==============================================================================
  // ProvokeNodeImpl class
  // ==============================================================================

  ProvokeNodeImpl::ProvokeNodeImpl(rclcpp::Node &node) :
    node_{node},
    timer_dispatch_{},
    timer_dispatches_{},
    tello_dispatches_{},
    base_machine_{base_machine::factory(*this)}
  {
    // Create a bunch of timer_dispatches for use by the par machine.
    // Give each its own identifier for logging.
    for (int i = 0; i < 4; i += 1) {
      tello_dispatches_.emplace_back(std::make_unique<TelloDispatch>(*this, i));
      timer_dispatches_.emplace_back(std::make_unique<TimerDispatch>(*this, i + 1, *tello_dispatches_[i]));
    }

    // Create the base timer dispatch
    timer_dispatch_ = std::make_unique<TimerDispatch>(*this, 0, *tello_dispatches_[0]);

    timer_ = node_.create_wall_timer(
      std::chrono::milliseconds{timer_interval_ms},
      [this]() -> void
      {
        // if the TelloDispatch objects aren't inited, then don't dispatch.
        if (inited_) {
          base_machine_->on_timer(node_.now());
          return;
        }

        // Set the init flag if all the telloDispatch classed are ready.
        for (auto &tello_dispatch : tello_dispatches_) {
          if (!tello_dispatch->is_action_client_ready()) {
            return;
          }
        }
        inited_ = true;
      });

    (void) timer_;
  }

  ProvokeNodeImpl::~ProvokeNodeImpl() = default;

  // ==============================================================================
  // ProvokeNode class
  // ==============================================================================

  class ProvokeNode : public rclcpp::Node
  {
    ProvokeNodeImpl impl_;

  public:
    explicit ProvokeNode(rclcpp::NodeOptions &options)
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
