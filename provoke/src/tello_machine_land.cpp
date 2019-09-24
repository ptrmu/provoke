
#include "tello_dispatch.hpp"
#include "tello_interface.hpp"

namespace provoke
{
  namespace tello_machine_land
  {
    class Machine : public TelloInterface
    {
      TelloDispatch &dispatch_;

    public:
      explicit Machine(TelloDispatch &dispatch)
        : TelloInterface{"tello_machine_land", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine() = default;

      Result on_timer(const rclcpp::Time &now) override
      {
        (void) now;
        return Result::conclusion();
      }

      Result validate_args(YamlArgs &args) override
      {
        (void) args;
        return Result::success();
      }

      Result prepare_from_args(YamlArgs &args) override
      {
        (void) args;

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Prepare %s:%s",
                    dispatch_.name_.c_str(), name_.c_str());

        return Result::success();
      }
    };

    std::unique_ptr<TelloInterface> factory(TelloDispatch &dispatch)
    {
      return std::unique_ptr<TelloInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}
