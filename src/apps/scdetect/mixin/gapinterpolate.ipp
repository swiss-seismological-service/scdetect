#ifndef SCDETECT_APPS_SCDETECT_MIXIN_GAPINTERPOLATE_IPP_
#define SCDETECT_APPS_SCDETECT_MIXIN_GAPINTERPOLATE_IPP_

#include <seiscomp/core/genericrecord.h>

#include "../log.h"
#include "../util/memory.h"

namespace Seiscomp {
namespace detect {

template <typename TGapInterpolateable>
InterpolateGaps<TGapInterpolateable>::~InterpolateGaps() {}

template <typename TGapInterpolateable>
void InterpolateGaps<TGapInterpolateable>::setGapInterpolation(
    bool gapInterpolation) {
  _gapInterpolation = gapInterpolation;
}

template <typename TGapInterpolateable>
bool InterpolateGaps<TGapInterpolateable>::gapInterpolation() const {
  return _gapInterpolation;
}

template <typename TGapInterpolateable>
void InterpolateGaps<TGapInterpolateable>::setGapThreshold(
    const Core::TimeSpan &duration) {
  _gapThreshold = duration;
}

template <typename TGapInterpolateable>
const Core::TimeSpan InterpolateGaps<TGapInterpolateable>::gapThreshold()
    const {
  return _gapThreshold;
}

template <typename TGapInterpolateable>
void InterpolateGaps<TGapInterpolateable>::setGapTolerance(
    const Core::TimeSpan &duration) {
  if (duration && duration > Core::TimeSpan{0.0}) {
    _gapTolerance = duration;
  }
}

template <typename TGapInterpolateable>
const Core::TimeSpan InterpolateGaps<TGapInterpolateable>::gapTolerance()
    const {
  return _gapTolerance;
}

template <typename TGapInterpolateable>
bool InterpolateGaps<TGapInterpolateable>::handleGap(StreamState &streamState,
                                                     const Record *record,
                                                     DoubleArrayPtr &data) {
  Core::TimeSpan gap{record->startTime() -
                     streamState.dataTimeWindow.endTime() -
                     /*one usec*/ Core::TimeSpan(0, 1)};
  double gapSeconds = static_cast<double>(gap);

  size_t gapSamples;
  if (gap > streamState.gapThreshold) {
    gapSamples =
        static_cast<size_t>(ceil(streamState.samplingFrequency * gapSeconds));
    if (fillGap(streamState, record, gap, (*data)[0], gapSamples)) {
      SCDETECT_LOG_DEBUG("%s: detected gap (%.6f secs, %lu samples) (handled)",
                         record->streamID().c_str(), gapSeconds, gapSamples);
    } else {
      SCDETECT_LOG_DEBUG(
          "%s: detected gap (%.6f secs, %lu samples) (NOT "
          "handled)",
          record->streamID().c_str(), gapSeconds, gapSamples);
    }
  } else if (gapSeconds < 0) {
    // handle record from the past
    gapSamples = static_cast<size_t>(
        ceil(-1 * streamState.samplingFrequency * gapSeconds));
    if (gapSamples > 1) return false;
  }

  return true;
}

template <typename TGapInterpolateable>
bool InterpolateGaps<TGapInterpolateable>::fillGap(
    StreamState &streamState, const Record *record,
    const Core::TimeSpan &duration, double nextSample, size_t missingSamples) {
  if (duration <= _gapTolerance) {
    if (_gapInterpolation) {
      auto filled{util::make_unique<GenericRecord>(
          record->networkCode(), record->stationCode(), record->locationCode(),
          record->channelCode(), streamState.lastRecord->endTime(),
          record->samplingFrequency())};

      auto dataPtr{util::make_smart<DoubleArray>(missingSamples)};
      double delta{nextSample - streamState.lastSample};
      double step{1. / static_cast<double>(missingSamples + 1)};
      double di = step;
      for (size_t i = 0; i < missingSamples; ++i, di += step) {
        const double value{streamState.lastSample + di * delta};
        dataPtr->set(i, value);
      }

      filled->setData(missingSamples, dataPtr->typedData(), Array::DOUBLE);
      fill(streamState, /*record=*/filled.release(), dataPtr);

      return true;
    }
  }

  return false;
}

template <typename TGapInterpolateable>
void InterpolateGaps<TGapInterpolateable>::setMinimumGapThreshold(
    StreamState &streamState, const Record *record, const std::string &logTag) {
  streamState.gapThreshold = _gapThreshold;

  const Core::TimeSpan minThres{2 * 1.0 / record->samplingFrequency()};
  if (minThres > streamState.gapThreshold) {
    const auto &tag{logTag.empty() ? record->streamID() : logTag};
    SCDETECT_LOG_WARNING_TAGGED(
        tag,
        "Gap threshold smaller than twice the sampling interval: %ld.%06lds > "
        "%ld.%06lds. Resetting gap threshold.",
        minThres.seconds(), minThres.microseconds(),
        streamState.gapThreshold.seconds(),
        streamState.gapThreshold.microseconds());

    streamState.gapThreshold = minThres;
  }
}

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MIXIN_GAPINTERPOLATE_IPP_
