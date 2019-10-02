
#include "tello_dispatch.hpp"
#include "tello_machine_action.hpp"

namespace provoke
{
  class TelloMachineConstAction : public TelloMachineAction
  {
    const std::string action_str_;
    const double timeout_sec_;

  public:
    explicit TelloMachineConstAction(TelloDispatch &dispatch, const std::string &action_str, double timeout_sec) :
      TelloMachineAction{std::string("tello_machine_").append((action_str)), dispatch},
      action_str_{action_str},timeout_sec_{timeout_sec}
    {}

    ~TelloMachineConstAction() override = default;

    Result validate_args(YamlArgs &args) override
    {
      (void) args;
      return Result::success();
    }

    Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
    {
      (void) args;

      RCLCPP_INFO(impl_.node_.get_logger(),
                  "Prepare %s:%s",
                  dispatch_.name_.c_str(), name_.c_str());

      return prepare_from_action_str(now, action_str_, timeout_sec_);
    }
  };

  namespace tello_machine_takeoff
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch)
    {
      return std::unique_ptr<TelloInterface>{std::make_unique<TelloMachineConstAction>(dispatch, "takeoff", 10.0)};
    }
  }

  namespace tello_machine_land
  {
    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch)
    {
      return std::unique_ptr<TelloInterface>{std::make_unique<TelloMachineConstAction>(dispatch, "land", 10.0)};
    }
  }
}
