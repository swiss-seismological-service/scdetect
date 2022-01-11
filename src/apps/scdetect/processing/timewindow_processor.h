#ifndef SCDETECT_APPS_SCDETECT_PROCESSING_TIMEWINDOWPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_PROCESSING_TIMEWINDOWPROCESSOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include "waveform_operator.h"
#include "waveform_processor.h"

namespace Seiscomp {
namespace detect {
namespace processing {

// `WaveformProcessor` implementation operating on a fixed time window
class TimeWindowProcessor : public WaveformProcessor {
 public:
  // Sets the time window for the data to be fed
  void setTimeWindow(const Core::TimeWindow &tw);
  // Returns the configured time window of the data to be fed
  const Core::TimeWindow &timeWindow() const;

  // Returns the time window including the safety margin
  const Core::TimeWindow &safetyTimeWindow() const;

  // Allows derived classes to compute the required time window
  virtual void computeTimeWindow();

 protected:
  bool store(const Record *record) override;

  bool enoughDataReceived(const StreamState &streamState) const override;

 private:
  // Time window for the data to be fed
  Core::TimeWindow _timeWindow;
  // Time window for the data to be fed including the (filter) initialization
  // time
  Core::TimeWindow _safetyTimeWindow;
};

}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_PROCESSING_TIMEWINDOWPROCESSOR_H_
