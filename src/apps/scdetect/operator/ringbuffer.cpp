#include "ringbuffer.h"

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/recordsequence.h>

#include <exception>
#include <memory>

#include "../log.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

RingBufferOperator::RingBufferOperator(WaveformProcessor *waveformProcessor)
    : _processor{waveformProcessor} {}

RingBufferOperator::RingBufferOperator(WaveformProcessor *waveformProcessor,
                                       Core::TimeSpan bufferSize)
    : _bufferSize{bufferSize}, _processor{waveformProcessor} {}

RingBufferOperator::RingBufferOperator(
    WaveformProcessor *waveformProcessor, Core::TimeSpan bufferSize,
    const std::vector<WaveformStreamID> &wfStreamIds)
    : _bufferSize{bufferSize}, _processor{waveformProcessor} {
  for (const auto &wfStreamId : wfStreamIds) {
    add(wfStreamId);
  }
}

void RingBufferOperator::setGapInterpolation(bool gapInterpolation) {
  _gapInterpolation = gapInterpolation;
}

bool RingBufferOperator::gapInterpolation() const { return _gapInterpolation; }

void RingBufferOperator::setGapThreshold(const Core::TimeSpan &duration) {
  if (duration && duration > Core::TimeSpan{0.0}) {
    for (auto &streamConfigPair : _streamConfigs) {
      auto &streamState{streamConfigPair.second.streamState};
      if (streamState.gapThreshold != duration) {
        reset(streamState);
      }
    }

    _gapThreshold = duration;
  }
}

const Core::TimeSpan RingBufferOperator::gapThreshold() const {
  return _gapThreshold;
}

void RingBufferOperator::setGapTolerance(const Core::TimeSpan &duration) {
  if (duration && duration > Core::TimeSpan{0.0}) {
    _gapTolerance = duration;
  }
}

const Core::TimeSpan RingBufferOperator::gapTolerance() const {
  return _gapTolerance;
}

WaveformProcessor::Status RingBufferOperator::feed(const Record *record) {
  if (record->sampleCount() == 0)
    return WaveformProcessor::Status::kWaitingForData;

  auto it{_streamConfigs.find(record->streamID())};
  if (it == _streamConfigs.end()) {
    return WaveformProcessor::Status::kInvalidStream;
  }

  if (store(it->second.streamState, record)) {
    WaveformOperator::store(record);

    return WaveformProcessor::Status::kInProgress;
  }

  return WaveformProcessor::Status::kError;
}

void RingBufferOperator::reset() { _streamConfigs.clear(); }

void RingBufferOperator::add(WaveformStreamID wfStreamId) {
  add(wfStreamId, _bufferSize);
}

void RingBufferOperator::add(WaveformStreamID wfStreamId,
                             Core::TimeSpan bufferSize) {
  if (_streamConfigs.find(wfStreamId) != _streamConfigs.end()) {
    return;
  }

  if (!bufferSize) {
    bufferSize = _bufferSize;
  }

  _streamConfigs.emplace(
      wfStreamId,
      StreamConfig{StreamState{}, std::make_shared<RingBuffer>(bufferSize)});
}

const std::shared_ptr<RingBuffer> &RingBufferOperator::get(
    WaveformStreamID wfStreamId) {
  return _streamConfigs.at(wfStreamId).streamBuffer;
}

bool RingBufferOperator::store(RingBufferOperator::StreamState &streamState,
                               const Record *record) {
  if (!record->data()) return false;

  DoubleArrayPtr data{
      dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE))};

  if (streamState.lastRecord) {
    if (record == streamState.lastRecord) {
      return false;
    } else if (record->samplingFrequency() != streamState.samplingFrequency) {
      SCDETECT_LOG_WARNING_PROCESSOR(
          _processor,
          "%s: buffering operator: sampling frequency changed, resetting "
          "stream (sfreq_record != sfreq_stream): %f != %f",
          record->streamID().c_str(), record->samplingFrequency(),
          streamState.samplingFrequency);
      reset(streamState);
    } else if (!handleGap(streamState, record, data)) {
      return false;
    }

    streamState.dataTimeWindow.setEndTime(record->endTime());
  }

  if (!streamState.lastRecord) {
    try {
      setupStream(streamState, record);
    } catch (std::exception &e) {
      SCDETECT_LOG_WARNING_PROCESSOR(_processor,
                                     "%s: Failed to setup stream: %s",
                                     record->streamID().c_str(), e.what());
      return false;
    }

    auto &buffer{_streamConfigs.at(record->streamID()).streamBuffer};
    buffer->clear();
  }

  streamState.lastSample = (*data)[data->size() - 1];
  streamState.lastRecord = record;

  return fill(streamState, record, data);
}

bool RingBufferOperator::handleGap(StreamState &streamState,
                                   const Record *record, DoubleArrayPtr &data) {
  if (record == streamState.lastRecord) return false;

  Core::TimeSpan gap{record->startTime() -
                     streamState.dataTimeWindow.endTime() -
                     /*one usec*/ Core::TimeSpan(0, 1)};
  double gapSeconds = static_cast<double>(gap);

  size_t gapSamples;
  if (gap > streamState.gapThreshold) {
    gapSamples =
        static_cast<size_t>(ceil(streamState.samplingFrequency * gapSeconds));
    if (fillGap(streamState, record, gap, (*data)[0], gapSamples)) {
      SCDETECT_LOG_DEBUG_PROCESSOR(
          _processor, "%s: detected gap (%.6f secs, %lu samples) (handled)",
          record->streamID().c_str(), gapSeconds, gapSamples);
    } else {
      SCDETECT_LOG_DEBUG_PROCESSOR(
          _processor,
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

bool RingBufferOperator::fill(StreamState &streamState, const Record *record,
                              DoubleArrayPtr &data) {
  auto &buffer{_streamConfigs.at(record->streamID()).streamBuffer};
  // buffer record
  auto retval{buffer->feed(record)};
  if (!retval) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        _processor,
        "%s: error while buffering data: start=%s, end=%s, samples=%d",
        record->streamID().c_str(), record->startTime().iso().c_str(),
        record->endTime().iso().c_str(), record->sampleCount());
  }
  return retval;
}

void RingBufferOperator::setupStream(StreamState &streamState,
                                     const Record *record) {
  const auto &f{record->samplingFrequency()};
  streamState.samplingFrequency = f;
  streamState.gapThreshold = _gapThreshold;

  const auto minThres{2 * 1.0 / f};
  if (minThres > streamState.gapThreshold) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        _processor,
        "Gap threshold smaller than twice the sampling interval: %fs > %fs. "
        "Resetting gap threshold.",
        minThres, static_cast<double>(streamState.gapThreshold));

    streamState.gapThreshold = minThres;
  }

  // update the received data timewindow
  streamState.dataTimeWindow = record->timeWindow();
}

void RingBufferOperator::reset(StreamState &streamState) {
  streamState.lastRecord.reset();
}

bool RingBufferOperator::fillGap(StreamState &streamState, const Record *record,
                                 const Core::TimeSpan &duration,
                                 double nextSample, size_t missingSamples) {
  if (duration <= _gapTolerance) {
    if (_gapInterpolation) {
      auto &buffer{_streamConfigs.at(record->streamID()).streamBuffer};

      auto filled{utils::make_unique<GenericRecord>(
          record->networkCode(), record->stationCode(), record->locationCode(),
          record->channelCode(), buffer->timeWindow().endTime(),
          record->samplingFrequency())};

      auto dataPtr{utils::make_smart<DoubleArray>(missingSamples)};
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

}  // namespace waveform_operator
}  // namespace detect
}  // namespace Seiscomp
