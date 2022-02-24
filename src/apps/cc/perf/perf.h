#ifndef SCDETECT_APPS_CC_PERF_PERF_H_
#define SCDETECT_APPS_CC_PERF_PERF_H_

#include <algorithm>
#include <boost/timer/timer.hpp>
#include <vector>

#include "../exception.h"

namespace Seiscomp {
namespace detect {
namespace perf {

class BaseException : public Exception {
 public:
  BaseException() : Exception{"performance base exception"} {}
  using Exception::Exception;
};

// A simple timer wrapper which records multiple time entries
class PerfTimer {
 public:
  using NanosecondType = boost::timer::nanosecond_type;

  PerfTimer() { _timer.stop(); }

  void start() { _timer.start(); }

  void stop() {
    _timer.stop();
    _times.push_back(_timer.elapsed().wall);
  }

  std::size_t trials() const { return _times.size(); }

  void clear() { _times.clear(); }

  NanosecondType lastTime() const {
    if (_times.empty()) {
      throw BaseException{"missing times"};
    }

    return _times.back();
  }

  NanosecondType minTime() const {
    if (_times.empty()) {
      throw BaseException{"missing times"};
    }
    return *std::min_element(_times.begin(), _times.end());
  }

  NanosecondType maxTime() const {
    if (_times.empty()) {
      throw BaseException{"missing times"};
    }
    return *std::max_element(_times.begin(), _times.end());
  }

 private:
  boost::timer::cpu_timer _timer;
  std::vector<NanosecondType> _times;
};

}  // namespace perf
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_PERF_PERF_H_
