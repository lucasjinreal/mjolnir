#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/types.h>


#define LOG_INFOF(...)                                                         \
  do {                                                                         \
    fprintf(stdout, "[I][%s:%s-%d]: ", __FILE__, __FUNCTION__, __LINE__);      \
    fprintf(stdout, __VA_ARGS__);                                              \
    fflush(NULL);                                                              \
  } while (0)

#define PRINTF(...)                                                            \
  do {                                                                         \
    fprintf(stdout, __VA_ARGS__);                                              \
    fflush(NULL);                                                              \
  } while (0)

class nullstream : public std::ostream {
public:
  nullstream()
      : std::ostream(nullptr) {} // results in rdbuf==0 and badbit==true
};
static nullstream dummyStream;

#define LOG(status) LOG_##status.stream()
#define LOG_ERROR LogMessage(__FILE__, __FUNCTION__, __LINE__, "E")
#define LOG_INFO LogMessage(__FILE__, __FUNCTION__, __LINE__, "I")
#define LOG_DEBUG LogMessage(__FILE__, __FUNCTION__, __LINE__, "D")
#define LOG_WARNING LogMessage(__FILE__, __FUNCTION__, __LINE__, "W")
#define LOG_FATAL LogMessageFatal(__FILE__, __FUNCTION__, __LINE__)
#define VLOG(level)                                                            \
  VLogMessage(__FILE__, __FUNCTION__, __LINE__, level).stream()

#define CHECK(x)                                                               \
  if (!(x))                                                                    \
  LogMessageFatal(__FILE__, __FUNCTION__, __LINE__).stream()                   \
      << "Check failed: " #x << ": " // NOLINT(*)

#define CHECK_EQ(x, y) _CHECK_BINARY(x, ==, y)
#define CHECK_NE(x, y) _CHECK_BINARY(x, !=, y)
#define CHECK_LT(x, y) _CHECK_BINARY(x, <, y)
#define CHECK_LE(x, y) _CHECK_BINARY(x, <=, y)
#define CHECK_GT(x, y) _CHECK_BINARY(x, >, y)
#define CHECK_GE(x, y) _CHECK_BINARY(x, >=, y)
#define _CHECK_BINARY(x, cmp, y) CHECK(x cmp y) << x << "!" #cmp << y << " "

inline std::tm localtime_xp(std::time_t timer) {
  std::tm bt{};
#if defined(__unix__)
  localtime_r(&timer, &bt);
#elif defined(_MSC_VER)
  localtime_s(&bt, &timer);
#else
  static std::mutex mtx;
  std::lock_guard<std::mutex> lock(mtx);
  bt = *std::localtime(&timer);
#endif
  return bt;
}

using namespace std;
inline void gen_log(std::ostream &log_stream_, const char *file,
                    const char *func, int lineno, const char *level,
                    const int kMaxLen = 20) {
  const int len = strlen(file);
  std::string time_str;

  std::time_t timestamp = std::time(nullptr);
  auto tm_time = localtime_xp(timestamp);

  log_stream_ << level << ' ' << 1 + tm_time.tm_mon << '/' << tm_time.tm_mday
              << ' ' << tm_time.tm_hour << ':' << tm_time.tm_min << ':'
              << std::setw(2) << tm_time.tm_sec << " ";

  if (len > kMaxLen) {
    log_stream_ << "..." << file + len - kMaxLen << " " << func << ":" << lineno
                << "] ";
  } else {
    log_stream_ << file << " " << func << ":" << lineno << "] ";
  }
}

class LogMessage {
public:
  LogMessage(const char *file, const char *func, int lineno,
             const char *level = "I") {

    if (*level == 'I') {
#if LOG_LEVEL <= 1
      gen_log(log_stream_, file, func, lineno, level);
#else
      is_pass = true;
#endif
    } else if (*level == 'D') {
#if LOG_LEVEL <= 2
      gen_log(log_stream_, file, func, lineno, level);
#else
      is_pass = true;
#endif
    } else if (*level == 'W') {
#if LOG_LEVEL <= 3
      gen_log(log_stream_, file, func, lineno, level);
#else
      is_pass = true;
#endif
    } else if (*level == 'E') {
      gen_log(log_stream_, file, func, lineno, level);
    }
  }
  ~LogMessage() {
    if (!is_pass) {
      log_stream_ << '\n';
      fprintf(stderr, "%s", log_stream_.str().c_str());
    }
  }
  std::ostream &stream() {
    if (is_pass) {
      return dummyStream;
    } else {
      return log_stream_;
    }
  }

protected:
  bool is_pass = false;
  std::stringstream log_stream_;
  LogMessage(const LogMessage &) = delete;
  void operator=(const LogMessage &) = delete;
};

class LogMessageFatal : public LogMessage {
public:
  LogMessageFatal(const char *file, const char *func, int lineno,
                  const char *level = "F")
      : LogMessage(file, func, lineno, level) {}
  ~LogMessageFatal() {
    log_stream_ << '\n';
    fprintf(stderr, "%s", log_stream_.str().c_str());
    abort();
  }
};
#define TIMER_START(_X)                                                        \
  uint64_t _X##_start;                                                         \
  do {                                                                         \
    struct timeval current;                                                    \
    gettimeofday(&current, NULL);                                              \
    _X##_start = current.tv_sec * 1000000 + current.tv_usec;                   \
  } while (0)

#define TIMER_STOP(_X)                                                         \
  uint64_t _X##_stop;                                                          \
  do {                                                                         \
    struct timeval current;                                                    \
    gettimeofday(&current, nullptr);                                           \
    _X##_stop = current.tv_sec * 1000000 + current.tv_usec;                    \
  } while (0)

#define TIMER_USEC(_X) (float(_X##_stop - _X##_start))
#define TIMER_MSEC(_X) (float(_X##_stop - _X##_start) / 1000.0f)
