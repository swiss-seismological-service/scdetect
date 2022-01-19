#ifndef SCDETECT_APPS_CC_PROCESSING_STREAM_H_
#define SCDETECT_APPS_CC_PROCESSING_STREAM_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/timewindow.h>

namespace Seiscomp {
namespace detect {
namespace processing {

struct StreamState {
  virtual ~StreamState();
  // Value of the last sample
  double lastSample{0};
  // The last record received
  RecordCPtr lastRecord;
  // The overall time window received
  Core::TimeWindow dataTimeWindow;
  // The sampling frequency of the stream
  double samplingFrequency{0};
  // The stream specific minimum gap length to detect a gap
  Core::TimeSpan gapThreshold;
};

}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_PROCESSING_STREAM_H_
