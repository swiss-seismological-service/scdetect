#include "timewindowprocessor.h"

namespace Seiscomp {
namespace detect {

TimeWindowProcessor::TimeWindowProcessor(const std::string &id)
    : WaveformProcessor{id} {}

void TimeWindowProcessor::setTimeWindow(const Core::TimeWindow &tw) {
  auto currentTimeWindow = _timeWindow;

  if (!tw) {
    _timeWindow = Core::TimeWindow{};
    _safetyTimeWindow = Core::TimeWindow{};
    reset();
    return;
  }
  _timeWindow = tw;
  _safetyTimeWindow.setStartTime(_timeWindow.startTime() - _initTime -
                                 _safetyMargin);
  _safetyTimeWindow.setEndTime(_timeWindow.endTime());
  reset();
}

const Core::TimeWindow &TimeWindowProcessor::timeWindow() const {
  return _timeWindow;
}

const Core::TimeWindow &TimeWindowProcessor::safetyTimeWindow() const {
  return _safetyTimeWindow;
}

void TimeWindowProcessor::setMargin(const Core::TimeSpan &margin) {
  _safetyMargin = margin;

  if (_timeWindow) {
    _safetyTimeWindow.setStartTime(_timeWindow.startTime() - _initTime -
                                   _safetyMargin);
    reset();
  }
}

const Core::TimeSpan &TimeWindowProcessor::margin() const {
  return _safetyMargin;
}

bool TimeWindowProcessor::store(const Record *record) {
  if (!record->timeWindow().overlaps(_safetyTimeWindow)) {
    if (status() > Status::kInProgress) {
      return false;
    }

    // Terminate the processor if a record arrives with a starttime later than
    // the requested time window
    if (record->startTime() > _safetyTimeWindow.endTime()) {
      setStatus(Status::kTerminated, 0.0);
    }

    return false;
  }

  return WaveformProcessor::store(record);
}

}  // namespace detect
}  // namespace Seiscomp
