#include "processor.h"

#include "log.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {

Processor::Processor(const std::string &id,
                     const boost::filesystem::path &debug_info_dir)
    : id_{id}, debug_info_dir_{debug_info_dir} {}

Processor::Result::~Result() {}

void Processor::enable() {
  if (enabled_)
    return;
  enabled_ = true;
}

void Processor::disable() {
  if (!enabled_)
    return;
  enabled_ = false;
}

bool Processor::enabled() const { return enabled_; }

const std::string Processor::id() const { return id_; }

void Processor::set_result_callback(PublishResultCallback callback) {
  result_callback_ = callback;
}

Processor::Status Processor::status() const { return status_; }

double Processor::status_value() const { return status_value_; }

const Core::TimeSpan Processor::init_time() const { return init_time_; }

bool Processor::finished() const { return Status::kInProgress < status_; }

const Core::TimeWindow &Processor::processed() const { return processed_; }

const boost::filesystem::path &Processor::debug_info_dir() const {
  return debug_info_dir_;
}

bool Processor::debug_mode() const { return !debug_info_dir_.empty(); }

void Processor::Reset() {
  status_ = Status::kWaitingForData;
  status_value_ = 0;

  processed_ = Core::TimeWindow{};
}

void Processor::Terminate() {
  set_status(Status::kTerminated, static_cast<int>(status_));
}

void Processor::Close() const {}

std::string Processor::DebugString() const { return ""; }

bool Processor::WithPicks() const { return false; }

Processor::StreamState::~StreamState() {
  if (filter) {
    delete filter;
  }
}

bool Processor::Store(StreamState &stream_state, RecordCPtr record) {
  if (Processor::Status::kInProgress < status() || !record->data())
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

  Fill(stream_state, record, data->size(), data->typedData());
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
  stream_state.last_sample = (*data)[data->size() - 1];

  return true;
}

bool Processor::HandleGap(StreamState &stream_state, RecordCPtr record,
                          DoubleArrayPtr data) {
  return true;
}

void Processor::Fill(StreamState &stream_state, RecordCPtr record, size_t n,
                     double *samples) {
  stream_state.received_samples += n;

  if (saturation_check_) {
    for (size_t i = 0; i < n; ++i) {
      if (fabs(samples[i]) >= saturation_threshold_) {
        set_status(Processor::Status::kDataClipped, samples[i]);
        break;
      }
    }
  }
  if (stream_state.filter)
    stream_state.filter->apply(n, samples);
}

bool Processor::EnoughDataReceived(const StreamState &stream_state) const {
  return stream_state.received_samples > stream_state.needed_samples;
}

void Processor::EmitResult(RecordCPtr record, ResultCPtr result) {
  if (enabled() && result_callback_)
    result_callback_(this, record, result);
}

void Processor::InitStream(StreamState &stream_state, RecordCPtr record) {
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

void Processor::set_status(Status status, double value) {
  status_ = status;
  status_value_ = value;
}

void Processor::set_debug_info_dir(const boost::filesystem::path &path) {
  debug_info_dir_ = path;
}

void Processor::set_processed(const Core::TimeWindow &tw) { processed_ = tw; }
void Processor::merge_processed(const Core::TimeWindow &tw) {
  processed_ = processed_ | tw;
}

void Processor::set_saturation_check(bool e) { saturation_check_ = e; }

bool Processor::saturation_check() const { return saturation_check_; }

void Processor::set_saturation_threshold(double thres) {
  saturation_threshold_ = thres;
}

namespace utils {

bool IsValidFilter(const std::string &filter_string, std::string &err) {
  auto filter{Processor::Filter::Create(filter_string, &err)};
  if (!filter) {
    return false;
  }
  delete filter;
  return true;
}
} // namespace utils

} // namespace detect
} // namespace Seiscomp
