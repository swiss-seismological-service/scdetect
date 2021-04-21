#include "waveformprocessor.h"

#include "log.h"
#include "utils.h"
#include "waveformoperator.h"

namespace Seiscomp {
namespace detect {

WaveformProcessor::WaveformProcessor(
    const std::string &id, const boost::filesystem::path &debug_info_dir)
    : Processor{id}, debug_info_dir_{debug_info_dir} {}

WaveformProcessor::Result::~Result() {}

void WaveformProcessor::enable() {
  if (enabled_)
    return;
  enabled_ = true;
}

void WaveformProcessor::disable() {
  if (!enabled_)
    return;
  enabled_ = false;
}

bool WaveformProcessor::enabled() const { return enabled_; }

void WaveformProcessor::set_result_callback(
    const PublishResultCallback &callback) {
  result_callback_ = callback;
}

WaveformProcessor::Status WaveformProcessor::status() const { return status_; }

double WaveformProcessor::status_value() const { return status_value_; }

void WaveformProcessor::set_operator(WaveformOperator *op) {
  if (waveform_operator_) {
    waveform_operator_.reset();
  }

  waveform_operator_.reset(op);
  if (waveform_operator_) {
    waveform_operator_->set_store_callback(
        [this](const Record *record) { return Store(record); });
  }
}

const Core::TimeSpan WaveformProcessor::init_time() const { return init_time_; }

bool WaveformProcessor::finished() const {
  return Status::kInProgress < status_;
}

const boost::filesystem::path &WaveformProcessor::debug_info_dir() const {
  return debug_info_dir_;
}

bool WaveformProcessor::debug_mode() const { return !debug_info_dir_.empty(); }

bool WaveformProcessor::Feed(const Record *record) {
  if (record->sampleCount() == 0)
    return false;

  if (!waveform_operator_) {
    return Store(record);
  }

  WaveformProcessor::Status s{waveform_operator_->Feed(record)};
  if (s > WaveformProcessor::Status::kTerminated) {
    set_status(s, -1);
    return false;
  }
  return true;
}

void WaveformProcessor::Reset() {
  if (waveform_operator_) {
    waveform_operator_->Reset();
  }

  status_ = Status::kWaitingForData;
  status_value_ = 0;
}

void WaveformProcessor::Terminate() {
  set_status(Status::kTerminated, static_cast<int>(status_));
}

void WaveformProcessor::Close() const {}

std::string WaveformProcessor::DebugString() const { return ""; }

WaveformProcessor::StreamState::~StreamState() {
  if (filter) {
    delete filter;
  }
}

bool WaveformProcessor::Store(const Record *record) {
  if (WaveformProcessor::Status::kInProgress < status() || !record->data())
    return false;

  try {
    StreamState &current_stream_state{stream_state(record)};

    DoubleArrayPtr data{
        dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE))};

    if (!current_stream_state.last_record) {
      InitStream(current_stream_state, record);
    } else {
      if (!HandleGap(current_stream_state, record, data))
        return false;

      current_stream_state.data_time_window.setEndTime(record->endTime());
    }
    current_stream_state.last_sample = (*data)[data->size() - 1];

    Fill(current_stream_state, record, data);
    if (Status::kInProgress < status())
      return false;

    if (!current_stream_state.initialized) {
      if (EnoughDataReceived(current_stream_state)) {
        // stream_state.initialized = true;
        Process(current_stream_state, record, *data);
        // NOTE: To allow derived classes to notice modification of the variable
        // stream_state.initialized, it is necessary to set this after calling
        // process.
        current_stream_state.initialized = true;
      }
    } else {
      // Call process to cause a derived processor to work on the data.
      Process(current_stream_state, record, *data);
    }

    current_stream_state.last_record = record;

  } catch (...) {
    return false;
  }

  return true;
}

bool WaveformProcessor::HandleGap(StreamState &stream_state,
                                  const Record *record, DoubleArrayPtr &data) {
  return true;
}

void WaveformProcessor::Fill(StreamState &stream_state, const Record *record,
                             DoubleArrayPtr &data) {

  const auto n{static_cast<size_t>(data->size())};
  stream_state.received_samples += n;

  if (stream_state.filter) {
    auto samples{data->typedData()};
    stream_state.filter->apply(n, samples);
  }
}

bool WaveformProcessor::EnoughDataReceived(
    const StreamState &stream_state) const {
  return stream_state.received_samples > stream_state.needed_samples;
}

void WaveformProcessor::EmitResult(const Record *record,
                                   const ResultCPtr &result) {
  if (enabled() && result_callback_)
    result_callback_(this, record, result);
}

void WaveformProcessor::InitStream(StreamState &stream_state,
                                   const Record *record) {
  const auto &f{record->samplingFrequency()};
  stream_state.sampling_frequency = f;
  stream_state.needed_samples = static_cast<size_t>(init_time_ * f + 0.5);
  if (stream_state.filter) {
    stream_state.filter->setSamplingFrequency(f);
  }

  // update the received data timewindow
  stream_state.data_time_window = record->timeWindow();

  if (stream_state.filter) {
    stream_state.filter->setStartTime(record->startTime());
    stream_state.filter->setStreamID(
        record->networkCode(), record->stationCode(), record->locationCode(),
        record->channelCode());
  }

  // update the received data timewindow
  stream_state.data_time_window = record->timeWindow();

  if (stream_state.filter) {
    stream_state.filter->setStartTime(record->startTime());
    stream_state.filter->setStreamID(
        record->networkCode(), record->stationCode(), record->locationCode(),
        record->channelCode());
  }
}

void WaveformProcessor::set_status(Status status, double value) {
  status_ = status;
  status_value_ = value;
}

void WaveformProcessor::set_debug_info_dir(
    const boost::filesystem::path &path) {
  debug_info_dir_ = path;
}

} // namespace detect
} // namespace Seiscomp
