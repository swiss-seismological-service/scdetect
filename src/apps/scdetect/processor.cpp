#include "processor.h"
#include "log.h"

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

const std::string &Processor::id() const { return id_; }

void Processor::set_result_callback(const PublishResultCallback &callback) {
  result_callback_ = callback;
}

Processor::Status Processor::status() const { return status_; }

double Processor::status_value() const { return status_value_; }

void Processor::set_gap_tolerance(const Core::TimeSpan &duration) {
  gap_tolerance_ = duration;
}

const Core::TimeSpan Processor::init_time() const { return init_time_; }

const Core::TimeSpan &Processor::gap_tolerance() const {
  return gap_tolerance_;
}

void Processor::set_gap_interpolation(bool e) { gap_interpolation_ = e; }

bool Processor::gap_interpolation() const { return gap_interpolation_; }

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

bool Processor::Store(StreamState &stream_state, RecordCPtr record) {
  if (Processor::Status::kInProgress < status() || !record->data())
    return false;

  DoubleArrayPtr data{
      dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE))};

  if (!HandleGap(stream_state, record, data))
    return false;

  // XXX: Do not use else here, because stream_state.last_record can be set to
  // nullptr when calling Reset() in FillGap(...)
  if (!stream_state.last_record) {
    InitFilter(stream_state, record->samplingFrequency());

    // update the received data timewindow
    stream_state.data_time_window = record->timeWindow();

    if (stream_state.filter) {
      stream_state.filter->setStartTime(record->startTime());
      stream_state.filter->setStreamID(
          record->networkCode(), record->stationCode(), record->locationCode(),
          record->channelCode());
    }
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
  if (stream_state.last_record) {
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
        SCDETECT_LOG_DEBUG(
            "%s: detected gap (%.6f secs, %lu samples) (handled)",
            record->streamID().c_str(), gap_seconds, gap_samples);
      } else {
        SCDETECT_LOG_DEBUG("%s: detected gap (%.6f secs, %lu samples) (NOT "
                           "handled): status=%d",
                           record->streamID().c_str(), gap_seconds, gap_samples,
                           // TODO(damb): Verify if this is the correct status
                           // to be displayed
                           static_cast<int>(status()));
        if (status() > Processor::Status::kInProgress)
          return false;
      }
    } else if (gap_seconds < 0) {
      // handle record from the past
      size_t gap_samples = static_cast<size_t>(
          ceil(-1 * stream_state.sampling_frequency * gap_seconds));
      if (gap_samples > 1)
        return false;
    }
    stream_state.data_time_window.setEndTime(record->endTime());
  }
  return true;
}

bool Processor::FillGap(StreamState &stream_state, RecordCPtr record,
                        const Core::TimeSpan &duration, double next_sample,
                        size_t missing_samples) {
  if (duration <= gap_tolerance_) {
    if (gap_interpolation_) {
      double delta{next_sample - stream_state.last_sample};
      double step{1. / static_cast<double>(missing_samples + 1)};
      double di = step;
      for (size_t i = 0; i < missing_samples; ++i, di += step) {
        double value{stream_state.last_sample + di * delta};
        Fill(stream_state, record, 1, &value);
      }
    }

    return true;
  }

  return false;
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

void Processor::InitFilter(StreamState &stream_state, double sampling_freq) {
  stream_state.sampling_frequency = sampling_freq;
  stream_state.needed_samples =
      static_cast<size_t>(init_time_ * stream_state.sampling_frequency + 0.5);
  if (stream_state.filter)
    stream_state.filter->setSamplingFrequency(sampling_freq);
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
