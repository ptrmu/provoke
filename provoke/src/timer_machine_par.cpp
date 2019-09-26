
#include "provoke_node_impl.hpp"
#include "timer_dispatch.hpp"
#include "yaml_args.hpp"

namespace provoke
{
  namespace timer_machine_par
  {
    // This machine has the following design:
    //  - An array of pointers to timer_dispatch instances from impl
    //  - on_timer() calls on_timer() of each non-null element of the array.
    //  - if individual on_timer returns concluded, then zero its entry in the array
    //  - if individual on_timer returns error, then call set_concluded on others and zero their entry in the array
    //  - in destructor, walk the array and call set_concluded on each non-zero
    //      entry -> This could happen when an error occurs in another part of the code
    //      and this machine gets destroyed. We have to ensure that the static timer_dispatch
    //      objects get cleaned up when this one gets cleaned up.

    class Machine : public ArgsInterface
    {
      TimerDispatch &dispatch_;

      std::vector<TimerDispatch *> par_dispatches_{};

    public:
      explicit Machine(TimerDispatch &dispatch)
        : ArgsInterface{"timer_machine_par", dispatch.impl_}, dispatch_{dispatch}
      {}

      ~Machine()
      {
        // Clean out any of the par_dispatches that haven't been cleaned up.
        // This could happen when an error occurs outside of this module and
        // this instance is freed.
        set_concluded();
      }

      void set_concluded()
      {
        for (auto &par_dispatch : par_dispatches_) {
          if (par_dispatch != nullptr) {
            par_dispatch->set_concluded();
            par_dispatch = nullptr;
          }
        }
        par_dispatches_.clear();
      }

      Result on_timer(const rclcpp::Time &now) override
      {
        // return concluded if all of the threads are concluded
        int active = 0;

        // Loop over the par_dispatches
        for (auto &par_dispatch : par_dispatches_) {
          if (par_dispatch != nullptr) {

            active += 1;

            // Call on_timer on each.
            auto result = par_dispatch->on_timer(now);

            // If the dispatch has concluded, then zero the entry.
            // It is already in the concluded state so we don't have
            // to call set_concluded().
            if (result.concluded()) {
              active -= 1;
              par_dispatch = nullptr;
            }

              // One task returned an error. So abort the others.
              // This might not be the best strategy, but lets try
              // it for now.
            else if (!result.succeeded()) {
              set_concluded();
              return result;
            }
          }
        }

        return active > 0 ? Result::success() : Result::conclusion();
      }

      Result validate_args(YamlArgs &args) override
      {
        // Par takes a sequence of sequences as the argument
        std::unique_ptr<YamlSeq> seq;
        auto result = args.get_seq(seq);
        if (!result.succeeded()) {
          return Result::make_result(ResultCodes::parse_error, "par machine requires a sequence argument.");
        }

        // Loop over the elements in the sequence
        while (!seq->done()) {
          // Get the args for this item.
          std::unique_ptr<YamlArgs> dispatch_args;
          result = seq->get_args(dispatch_args);
          if (!result.succeeded()) {
            return Result::make_result(ResultCodes::parse_error, "par machine can not get an argument.");
          }

          // See if the dispatcher validates the list.
          result = impl_.timer_dispatches_[0]->validate_args(*dispatch_args);
          if (!result.succeeded()) {
            return result;
          }

          // Move to the next element in the sequence
          seq->next();
        }

        return Result::success();
      }

      Result prepare_from_args(const rclcpp::Time &now, YamlArgs &args) override
      {
        // Probably not necessary but just in case, clear out any leftover state.
        set_concluded();

        // Par takes a sequence of sequences as the argument. This
        // should not error out because it was validated, but be safe.
        std::unique_ptr<YamlSeq> seq;
        auto result = args.get_seq(seq);
        if (!result.succeeded()) {
          return result;
        }

        RCLCPP_INFO(impl_.node_.get_logger(),
                    "Prepare %s:%s",
                    dispatch_.name_.c_str(), name_.c_str());

        // Walk through the available dispatches in impl and prepare them.
        // Again an error shouldn't happen because this has been validated.
        if (!seq->done()) {
          for (auto &timer_dispatch : impl_.timer_dispatches_) {
            std::unique_ptr<YamlArgs> dispatch_args;
            result = seq->get_args(dispatch_args);
            if (!result.succeeded()) {
              return Result::make_result(ResultCodes::parse_error, "par machine can not get an argument. (prepare)");
            }

            // Prepare this dispatch. If an error occurs, break out and clean up everything.
            result = timer_dispatch->prepare_from_args(now, *dispatch_args);
            if (!result.succeeded()) {
              break;
            }

            // Save a pointer to this dispatch in this object's par_dispatch list.
            par_dispatches_.emplace_back(timer_dispatch.get());

            // advance to the next timer_dispatch argument
            seq->next();

            // If there are no more tasks, then break from this loop
            if (seq->done()) {
              break;
            }
          }
        }

        // if there was an error clean up
        if (!result.succeeded()) {
          set_concluded();
          return result;
        }

        // If there were more arguments than timer_dispatches, then clean up
        if (!seq->done()) {
          set_concluded();
          return Result::make_result(ResultCodes::failure,
                                     "Machine - %s error: more sequences than dispatches.");
        }

        return Result::success();
      }
    };

    std::unique_ptr<ArgsInterface> factory(TimerDispatch &dispatch)
    {
      return std::unique_ptr<ArgsInterface>{std::make_unique<Machine>(dispatch)};
    }
  }
}

