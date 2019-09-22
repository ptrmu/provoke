
#include "result.hpp"

#include <stdarg.h> // for va_list, va_start

namespace provoke
{
  Result Result::make_result(ResultCodes code, std::string fmt_str, ...)
  {
    constexpr size_t string_reserve = 32;
    size_t str_len = std::max(fmt_str.size(), string_reserve);
    std::string str;

    do {
      va_list ap;
      va_start(ap, fmt_str); // NOTE: vsnprintf modifies ap so it has to be initialized in the loop

      str.resize(str_len);

      auto final_n = vsnprintf(const_cast<char *>(str.data()), str_len, fmt_str.c_str(), ap);

      // For an encoding error just return what is in the buffer.
      if (final_n < 0) {
        break;
      }

      // If the buffer sufficient, resize it and finish.
      if (final_n < static_cast<int>(str_len)) {
        str.resize(final_n + 1); // don't truncate the trailing null!
        break;
      }

      // The buffer was not large enough. So resize it.
      str_len = final_n + 1;

      va_end(ap);
    } while (true);

    return Result{code, str};
  }
}
