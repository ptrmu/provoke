#ifndef PROVOKE_NODE_IMPL_HPP
#define PROVOKE_NODE_IMPL_HPP

#include <memory>

#include "ros2_shared/context_macros.hpp"
#include "state_machine_interface.hpp"


namespace rclcpp
{
  class Node;

  class TimerBase;
}

namespace provoke
{
  class TimerDispatch;

  class TelloDispatch;

  class TimerInterface;

  // ==============================================================================
  // Parameters
  // ==============================================================================

#define ERR "[pause: 1, par: [[tello: [land]], [tello: [land]]]]"
#define PK0 "[tello: [land]]"
#define PK1 "[par: [[pause: 2], [pause: 3, pause: 4]]]"
#define PK2 "[tello: [takeoff, send: go 50 50 100 50, send: go 0 -100 0 50, send: go 0 50 -100 50, send: go 0 50 100 50, send: go 0 -100 0 50, send: go 0 50 -100 50, land]]"
#define PK3 "[par: [[tello: [takeoff]], [tello: [takeoff]]], par: [[tello: [send: go 50 0 0 50]], [tello: [send: go 50 0 0 50]]], par: [[tello: [send: go 0 50 0 50]], [tello: [send: go 0 50 0 50]]], par: [[tello: [send: go 0 -100 0 50]], [tello: [send: go 0 -100 0 50]]], par: [[tello: [send: go 0 50 0 50]], [tello: [send: go 0 50 0 50]]], par: [[tello: [land]], [tello: [land]]]]"
#define PK4 "[tello: [takeoff, send: go 50 0 0 50, send: go 0 50 0 50, send: go 0 -100 0 50, send: go 0 50 0 50, land]]"

// This is a slightly ugly way to initialize a vector of strings with the context macros.
  static std::vector<std::string> ns_drones_default{std::string{"drone_1"}, std::string{"drone_2"}};
//  static std::vector<std::string> ns_drones_default{};

#define PROVOKE_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(cmds_go, int, 0) /* poke list to execute */\
  CXT_MACRO_MEMBER(cmds_generate, std::string, "rot_oscillate_2") /* programmatically generate sequence */ \
  CXT_MACRO_MEMBER(cmds_err, std::string, ERR) /* Sequence of commands in respond to error */ \
  CXT_MACRO_MEMBER(cmds_0, std::string, PK0) /* Sequence of commands 0 */ \
  CXT_MACRO_MEMBER(cmds_1, std::string, PK1) /* Sequence of commands 1 */ \
  CXT_MACRO_MEMBER(cmds_2, std::string, PK2) /* Sequence of commands 2 */ \
  CXT_MACRO_MEMBER(cmds_3, std::string, PK3) /* Sequence of commands 3 */ \
  CXT_MACRO_MEMBER(cmds_4, std::string, PK4) /* Sequence of commands 4 */ \
  \
  CXT_MACRO_MEMBER(ns_drones, std::vector<std::string>, ns_drones_default) /* list of namespaces */\
  /* End of list */


  struct ParameterContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    PROVOKE_NODE_ALL_PARAMS
  };

  // ==============================================================================
  // class ProvokeNodeImpl
  // ==============================================================================

  class ProvokeNodeImpl
  {
  public:
    rclcpp::Node &node_;
    bool inited_{false};
    int init_try_count_{0};
    const long timer_interval_ms = 10;
    std::unique_ptr<TimerDispatch> timer_dispatch_;
    std::vector<std::unique_ptr<TimerDispatch>> timer_dispatches_;
    std::vector<std::unique_ptr<TelloDispatch>> tello_dispatches_;

    ParameterContext cxt_;

  private:
    std::unique_ptr<TimerInterface> base_machine_;
    std::shared_ptr<rclcpp::TimerBase> timer_{};

    void validate_parameters();

  public:
    explicit ProvokeNodeImpl(rclcpp::Node &node);

    ~ProvokeNodeImpl();
  };
}

#endif //PROVOKE_NODE_IMPL_HPP
