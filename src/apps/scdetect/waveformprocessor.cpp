#include "waveformprocessor.h"

#include "log.h"
#include "utils.h"

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

const Core::TimeSpan WaveformProcessor::init_time() const { return init_time_; }

bool WaveformProcessor::finished() const {
  return Status::kInProgress < status_;
}

const boost::filesystem::path &WaveformProcessor::debug_info_dir() const {
  return debug_info_dir_;
}

bool WaveformProcessor::debug_mode() const { return !debug_info_dir_.empty(); }

void WaveformProcessor::Reset() {
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

bool WaveformProcessor::Store(StreamState &stream_state, const Record *record) {
  if (WaveformProcessor::Status::kInProgress < status() || !record->data())
    return false;

  DoubleArrayPtr data{
      dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE))};

  if (!stream_state.last_record) {
    InitStream(stream_state, record);
  } else {
    if (!HandleGap(stream_state, record, data))
      return false;

    stream_state.data_time_window.setEndTime(record->endTime());
  }
  stream_state.last_sample = (*data)[data->size() - 1];

  Fill(stream_state, record, data);
  if (Status::kInProgress < status())
    return false;

  if (!stream_state.initialized) {
    if (EnoughDataReceived(stream_state)) {
      // stream_state.initialized = true;
      Process(stream_state, record, *data);
      // NOTE: To allow derived classes to notice modification of the variable
      // stream_state.initialized, it is necessary to set this after calling
      // process.
      stream_state.initialized = true;
    }
  } else {
    // Call process to cause a derived processor to work on the data.
    Process(stream_state, record, *data);
  }

  stream_state.last_record = record;

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

  auto samples{data->typedData()};
  if (saturation_check_) {
    for (size_t i = 0; i < n; ++i) {
      if (fabs(samples[i]) >= saturation_threshold_) {
        set_status(WaveformProcessor::Status::kDataClipped, samples[i]);
        break;
      }
    }
  }
  if (stream_state.filter)
    stream_state.filter->apply(n, samples);
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

void WaveformProcessor::set_saturation_check(bool e) { saturation_check_ = e; }

bool WaveformProcessor::saturation_check() const { return saturation_check_; }

void WaveformProcessor::set_saturation_threshold(double thres) {
  saturation_threshold_ = thres;
}

} // namespace detect
} // namespace Seiscomp
