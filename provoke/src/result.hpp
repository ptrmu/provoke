
#ifndef RESULT_HPP
#define RESULT_HPP

#include <string>

namespace provoke
{
  enum class ResultCodes
  {
    success = 0,
    conclusion,
    timeout,
    logic_error,
    failure,
  };


  class Result
  {
    ResultCodes code_;
    std::string msg_;

  public:
    Result() :
      code_{ResultCodes::success}, msg_{}
    {
    }

    Result(ResultCodes code, const std::string &msg) :
      code_{code}, msg_{msg}
    {}

    static Result make_result(ResultCodes code, std::string fmt_str, ...);

    auto code()
    { return code_; }

    auto &msg()
    { return msg_; }

    bool succeeded()
    { return code_ == ResultCodes::success; }

    bool concluded()
    { return code_ == ResultCodes::conclusion; }

    static Result success()
    {
      return Result{};
    }

    static Result conclusion()
    {
      return Result{ResultCodes::conclusion, ""};
    }

    static Result failure()
    {
      return Result{ResultCodes::failure, ""};
    }
  };
}
#endif //RESULT_HPP
