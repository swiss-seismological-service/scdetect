#include "gap_interpolate.h"

#include <seiscomp/core/genericrecord.h>

#include <cmath>

#include "../../log.h"
#include "../../util/memory.h"

namespace Seiscomp {
namespace detect {
namespace processing {
namespace detail {

void InterpolateGaps::setGapInterpolation(bool gapInterpolation) {
  _gapInterpolation = gapInterpolation;
}

bool InterpolateGaps::gapInterpolation() const { return _gapInterpolation; }

void InterpolateGaps::setGapThreshold(const Core::TimeSpan &duration) {
  _gapThreshold = duration;
}

const Core::TimeSpan &InterpolateGaps::gapThreshold() const {
  return _gapThreshold;
}

void InterpolateGaps::setGapTolerance(const Core::TimeSpan &duration) {
  if (duration && duration > Core::TimeSpan{0.0}) {
    _gapTolerance = duration;
  }
}

const Core::TimeSpan &InterpolateGaps::gapTolerance() const {
  return _gapTolerance;
}

bool InterpolateGaps::handleGap(StreamState &streamState, const Record *record,
                                DoubleArrayPtr &data) {
  Core::TimeSpan gap{record->startTime() -
                     streamState.dataTimeWindow.endTime() -
                     /*one usec*/ Core::TimeSpan(0, 1)};
  auto gapSeconds{static_cast<double>(gap)};

  std::size_t gapSamples{0};
  if (gap > streamState.gapThreshold) {
    gapSamples = std::ceil(streamState.samplingFrequency * gapSeconds);
    if (fillGap(streamState, record, gap, (*data)[0], gapSamples)) {
      SCDETECT_LOG_DEBUG("%s: detected gap (%.6f secs, %lu samples) (handled)",
                         record->streamID().c_str(), gapSeconds, gapSamples);
    } else {
      SCDETECT_LOG_DEBUG(
          "%s: detected gap (%.6f secs, %lu samples) (NOT handled)",
          record->streamID().c_str(), gapSeconds, gapSamples);
    }
  } else if (gapSeconds < 0) {
    // handle record from the past
    gapSamples = std::ceil(-1 * streamState.samplingFrequency * gapSeconds);
    if (gapSamples > 1) {
      return false;
    }
  }

  return true;
}

bool InterpolateGaps::fillGap(StreamState &streamState, const Record *record,
                              const Core::TimeSpan &duration, double nextSample,
                              size_t missingSamples) {
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

void InterpolateGaps::setMinimumGapThreshold(StreamState &streamState,
                                             const Record *record,
                                             const std::string &logTag) {
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

}  // namespace detail
}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp
