#ifndef SCDETECT_APPS_SCDETECT_MIXIN_GAPINTERPLATE_H_
#define SCDETECT_APPS_SCDETECT_MIXIN_GAPINTERPLATE_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/typedarray.h>

#include "../stream.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {

// Mixin which provides facilities for gap interpolation
template <typename TGapInterpolateable>
class InterpolateGaps {
 public:
  virtual ~InterpolateGaps();
  // Enables/disables the linear interpolation of missing samples
  // if the gap is smaller than the configured gap tolerance
  void setGapInterpolation(bool gapInterpolation);
  // Returns if gap interpolation is enabled or disabled, respectively
  bool gapInterpolation() const;
  // Sets the threshold (i.e. the minimum gap length) for gap interpolation
  virtual void setGapThreshold(const Core::TimeSpan &duration);
  // Returns the gap threshold configured
  const Core::TimeSpan gapThreshold() const;
  // Sets the maximum gap length to be tolerated
  void setGapTolerance(const Core::TimeSpan &duration);
  // Returns the gap tolerance configured
  const Core::TimeSpan gapTolerance() const;

 protected:
  // Fill data
  virtual bool fill(StreamState &streamState, const Record *record,
                    DoubleArrayPtr &data) = 0;

  bool handleGap(StreamState &streamState, const Record *record,
                 DoubleArrayPtr &data);
  // Fill gaps
  virtual bool fillGap(StreamState &streamState, const Record *record,
                       const Core::TimeSpan &duration, double nextSample,
                       size_t missingSamples);

  // Sets the `streamState` specific minimum gap length
  void setMinimumGapThreshold(StreamState &streamState, const Record *record,
                              const std::string &logTag = "");

  // The configured minimum gap length to detect a gap
  Core::TimeSpan _gapThreshold;

 private:
  InterpolateGaps() = default;
  friend TGapInterpolateable;

  // Indicates if gap interpolation is enabled/disabled
  bool _gapInterpolation{false};
  // The maximum gap length to tolerate
  Core::TimeSpan _gapTolerance;
};

}  // namespace detect
}  // namespace Seiscomp

#include "gapinterpolate.ipp"

#endif  // SCDETECT_APPS_SCDETECT_MIXIN_GAPINTERPLATE_H_
