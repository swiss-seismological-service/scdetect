#include "ringbuffer.h"

#include <exception>
#include <memory>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/recordsequence.h>

#include "../log.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

RingBufferOperator::RingBufferOperator(WaveformProcessor *waveform_processor)
    : waveform_processor_{waveform_processor} {}

RingBufferOperator::RingBufferOperator(WaveformProcessor *waveform_processor,
                                       Core::TimeSpan buffer_size)
    : buffer_size_{buffer_size}, waveform_processor_{waveform_processor} {}

RingBufferOperator::RingBufferOperator(
    WaveformProcessor *waveform_processor, Core::TimeSpan buffer_size,
    const std::vector<WaveformStreamID> &wf_stream_ids)
    : buffer_size_{buffer_size}, waveform_processor_{waveform_processor} {

  for (const auto &wf_stream_id : wf_stream_ids) {
    Add(wf_stream_id);
  }
}

void RingBufferOperator::set_gap_interpolation(bool gap_interpolation) {
  gap_interpolation_ = gap_interpolation;
}

bool RingBufferOperator::gap_interpolation() const {
  return gap_interpolation_;
}

void RingBufferOperator::set_gap_threshold(const Core::TimeSpan &duration) {
  if (duration && duration > Core::TimeSpan{0.0}) {
    gap_threshold_ = duration;
  }
}

const Core::TimeSpan RingBufferOperator::gap_threshold() const {
  return gap_threshold_;
}

void RingBufferOperator::set_gap_tolerance(const Core::TimeSpan &duration) {
  if (duration && duration > Core::TimeSpan{0.0}) {
    gap_tolerance_ = duration;
  }
}

const Core::TimeSpan RingBufferOperator::gap_tolerance() const {
  return gap_tolerance_;
}

WaveformProcessor::Status RingBufferOperator::Feed(const Record *record) {
  if (record->sampleCount() == 0)
    return WaveformProcessor::Status::kWaitingForData;

  auto it{stream_configs_.find(record->streamID())};
  if (it == stream_configs_.end()) {
    return WaveformProcessor::Status::kInvalidStream;
  }

  if (Store(it->second.stream_state, record)) {
    WaveformOperator::Store(record);

    return WaveformProcessor::Status::kInProgress;
  }

  return WaveformProcessor::Status::kError;
}

void RingBufferOperator::Reset() { stream_configs_.clear(); }

void RingBufferOperator::Add(WaveformStreamID wf_stream_id) {
  Add(wf_stream_id, buffer_size_);
}

void RingBufferOperator::Add(WaveformStreamID wf_stream_id,
                             Core::TimeSpan buffer_size) {
  if (stream_configs_.find(wf_stream_id) != stream_configs_.end()) {
    return;
  }

  if (!buffer_size) {
    buffer_size = buffer_size_;
  }

  stream_configs_.emplace(
      wf_stream_id,
      StreamConfig{StreamState{}, std::make_shared<RingBuffer>(buffer_size)});
}

const std::shared_ptr<RingBuffer> &
RingBufferOperator::Get(WaveformStreamID wf_stream_id) {
  return stream_configs_.at(wf_stream_id).stream_buffer;
}

bool RingBufferOperator::Store(RingBufferOperator::StreamState &stream_state,
                               const Record *record) {

  if (!record->data())
    return false;

  DoubleArrayPtr data{
      dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE))};

  if (stream_state.last_record) {
    if (record == stream_state.last_record) {
      return false;
    } else if (record->samplingFrequency() != stream_state.sampling_frequency) {
      SCDETECT_LOG_WARNING_PROCESSOR(
          waveform_processor_,
          "%s: buffering operator: sampling frequency changed, resetting "
          "stream: %f != %f",
          record->streamID().c_str(), record->samplingFrequency(),
          stream_state.sampling_frequency);
      stream_state.last_record.reset();
    } else if (!HandleGap(stream_state, record, data)) {
      return false;
    }

    stream_state.data_time_window.setEndTime(record->endTime());
  }

  if (!stream_state.last_record) {
    try {
      SetupStream(stream_state, record);
    } catch (std::exception &e) {
      SCDETECT_LOG_WARNING_PROCESSOR(waveform_processor_,
                                     "%s: Failed to setup stream: %s",
                                     record->streamID().c_str(), e.what());
      return false;
    }

    auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};
    buffer->clear();
  }

  stream_state.last_sample = (*data)[data->size() - 1];
  stream_state.last_record = record;

  return Fill(stream_state, record, data);
}

bool RingBufferOperator::HandleGap(StreamState &stream_state,
                                   const Record *record, DoubleArrayPtr &data) {

  if (record == stream_state.last_record)
    return false;

  Core::TimeSpan gap{record->startTime() -
                     stream_state.data_time_window.endTime() -
                     /* one usec*/ Core::TimeSpan(0, 1)};
  double gap_seconds = static_cast<double>(gap);

  if (gap > gap_threshold_) {
    size_t gap_samples = static_cast<size_t>(
        ceil(stream_state.sampling_frequency * gap_seconds));
    if (FillGap(stream_state, record, gap, (*data)[0], gap_samples)) {
      SCDETECT_LOG_DEBUG_PROCESSOR(
          waveform_processor_,
          "%s: detected gap (%.6f secs, %lu samples) (handled)",
          record->streamID().c_str(), gap_seconds, gap_samples);
    } else {
      SCDETECT_LOG_DEBUG_PROCESSOR(
          waveform_processor_,
          "%s: detected gap (%.6f secs, %lu samples) (NOT "
          "handled)",
          record->streamID().c_str(), gap_seconds, gap_samples);
    }
  } else if (gap_seconds < 0) {
    // handle record from the past
    size_t gap_samples = static_cast<size_t>(
        ceil(-1 * stream_state.sampling_frequency * gap_seconds));
    if (gap_samples > 1)
      return false;
  }

  return true;
}

bool RingBufferOperator::Fill(StreamState &stream_state, const Record *record,
                              DoubleArrayPtr &data) {
  auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};
  // buffer record
  auto retval{buffer->feed(record)};
  if (!retval) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        waveform_processor_,
        "%s: error while buffering data: start=%s, end=%s, samples=%d",
        record->streamID().c_str(), record->startTime().iso().c_str(),
        record->endTime().iso().c_str(), record->sampleCount());
  }
  return retval;
}

void RingBufferOperator::SetupStream(StreamState &stream_state,
                                     const Record *record) {
  const auto &f{record->samplingFrequency()};
  stream_state.sampling_frequency = f;

  const auto min_thres{2 * 1.0 / f};
  if (min_thres > gap_threshold_) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        waveform_processor_,
        "Gap threshold smaller than twice the sampling interval: %fs > %fs. "
        "Resetting gap threshold.",
        min_thres, static_cast<double>(gap_threshold_));

    // TODO(damb): When implementing the feature/handle-changing-sampling
    // rates (see: https://github.com/damb/scdetect/issues/20) store remember
    // the configured gap threshold value and reset the current gap threshold
    // to the configured one, once the sampling interval decreases.
    gap_threshold_ = min_thres;
  }

  // update the received data timewindow
  stream_state.data_time_window = record->timeWindow();
}

bool RingBufferOperator::FillGap(StreamState &stream_state,
                                 const Record *record,
                                 const Core::TimeSpan &duration,
                                 double next_sample, size_t missing_samples) {
  if (duration <= gap_tolerance_) {
    if (gap_interpolation_) {

      auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};

      auto filled{utils::make_unique<GenericRecord>(
          record->networkCode(), record->stationCode(), record->locationCode(),
          record->channelCode(), buffer->timeWindow().endTime(),
          record->samplingFrequency())};

      auto data_ptr{utils::make_smart<DoubleArray>(missing_samples)};
      double delta{next_sample - stream_state.last_sample};
      double step{1. / static_cast<double>(missing_samples + 1)};
      double di = step;
      for (size_t i = 0; i < missing_samples; ++i, di += step) {
        const double value{stream_state.last_sample + di * delta};
        data_ptr->set(i, value);
      }

      filled->setData(missing_samples, data_ptr->typedData(), Array::DOUBLE);
      Fill(stream_state, /*record=*/filled.release(), data_ptr);

      return true;
    }
  }

  return false;
}

} // namespace waveform_operator
} // namespace detect
} // namespace Seiscomp
