#ifndef SCDETECT_APPS_CC_PROCESSING_DETAIL_GAPINTERPOLATE_H_
#define SCDETECT_APPS_CC_PROCESSING_DETAIL_GAPINTERPOLATE_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/typedarray.h>

#include "../stream.h"

namespace Seiscomp {
namespace detect {
namespace processing {
namespace detail {

// Mixin like class which provides facilities for gap interpolation
class InterpolateGaps {
 public:
  virtual ~InterpolateGaps() = default;
  // Enables/disables the linear interpolation of missing samples
  // if the gap is smaller than the configured gap tolerance
  virtual void setGapInterpolation(bool gapInterpolation);
  // Returns if gap interpolation is enabled or disabled, respectively
  bool gapInterpolation() const;
  // Sets the threshold (i.e. the minimum gap length) for gap interpolation
  virtual void setGapThreshold(const Core::TimeSpan &duration);
  // Returns the gap threshold configured
  const Core::TimeSpan &gapThreshold() const;
  // Sets the maximum gap length to be tolerated
  virtual void setGapTolerance(const Core::TimeSpan &duration);
  // Returns the gap tolerance configured
  const Core::TimeSpan &gapTolerance() const;

 protected:
  virtual bool fill(processing::StreamState &streamState, const Record *record,
                    DoubleArrayPtr &data) = 0;

  virtual bool handleGap(StreamState &streamState, const Record *record,
                         DoubleArrayPtr &data);

  // Sets the `streamState` specific minimum gap length
  void setMinimumGapThreshold(StreamState &streamState, const Record *record,
                              const std::string &logTag = "");

 private:
  // Fill gaps
  bool fillGap(StreamState &streamState, const Record *record,
               const Core::TimeSpan &duration, double nextSample,
               size_t missingSamples);

  // The configured minimum gap length to detect a gap
  Core::TimeSpan _gapThreshold;
  // The maximum gap length to tolerate
  Core::TimeSpan _gapTolerance;
  // Indicates if gap interpolation is enabled/disabled
  bool _gapInterpolation{false};
};

}  // namespace detail
}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_PROCESSING_DETAIL_GAPINTERPOLATE_H_
