
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
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(PROVOKE_NODE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node_, PROVOKE_NODE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, node_.get_logger(), cxt_, n, t, d)
    PROVOKE_NODE_ALL_PARAMS

    // If there are no namespaces, then create a vector with one empty namespace
    auto ns_drones = cxt_.ns_drones_;
    if (ns_drones.empty()) {
      ns_drones.emplace_back(std::string{});
    }

    // Create a bunch of timer_dispatches for use by the par machine.
    // Give each its own identifier for logging.
    for (size_t i = 0; i < ns_drones.size(); i += 1) {

      tello_dispatches_.emplace_back(std::make_unique<TelloDispatch>(*this, i, ns_drones[i]));
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

        init_try_count_ += 1;
        // Set the init flag if all the telloDispatch classed are ready.
        for (auto &tello_dispatch : tello_dispatches_) {
          if (!tello_dispatch->is_action_client_ready()) {
            if (init_try_count_ > 200) {
              init_try_count_ = 0;
              RCLCPP_INFO(node_.get_logger(),
                          "%s action_client_ready() returns false. Continuing to try,",
                          tello_dispatch->name_.c_str());
            }
            return;
          }
        }
        inited_ = true;
      });
    (void) timer_;
  }

  ProvokeNodeImpl::~ProvokeNodeImpl() = default;

  void ProvokeNodeImpl::validate_parameters()
  {
    (void) this; // silence static warning
  }

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
