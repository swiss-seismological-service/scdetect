#include "template.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "../log.h"
#include "../operator/resample.h"
#include "../resamplerstore.h"
#include "../settings.h"
#include "../utils.h"
#include "../waveform.h"
#include "../waveformoperator.h"

namespace Seiscomp {
namespace detect {
namespace detector {

Template::Template(const GenericRecordCPtr &waveform,
                   const std::string filter_id,
                   const Core::Time &template_starttime,
                   const Core::Time &template_endtime,
                   const std::string &processor_id, const Processor *p)
    : WaveformProcessor{p ? std::string{p->id() + settings::kProcessorIdSep +
                                        processor_id}
                          : processor_id},
      cross_correlation_{waveform, filter_id, template_starttime,
                         template_endtime} {}

void Template::set_filter(Filter *filter, const Core::TimeSpan &init_time) {
  if (stream_state_.filter) delete stream_state_.filter;

  stream_state_.filter = filter;
  init_time_ =
      std::max(init_time, Core::TimeSpan{cross_correlation_.template_length()});
}

const Core::TimeWindow &Template::processed() const {
  return stream_state_.data_time_window;
}

void Template::Reset() {
  Filter *tmp{stream_state_.filter};

  stream_state_ = StreamState{};
  if (tmp) {
    stream_state_.filter = tmp->clone();
    delete tmp;
  }

  cross_correlation_.Reset();

  WaveformProcessor::Reset();
}

void Template::set_target_sampling_frequency(double f) {
  if (f > 0) {
    target_sampling_frequency_ = f;
  }
}

boost::optional<double> Template::target_sampling_frequency() const {
  return target_sampling_frequency_;
}

boost::optional<const Core::Time> Template::template_starttime() const {
  return cross_correlation_.template_starttime();
}

boost::optional<const Core::Time> Template::template_endtime() const {
  return cross_correlation_.template_endtime();
}

WaveformProcessor::StreamState &Template::stream_state(const Record *record) {
  return stream_state_;
}

void Template::Process(StreamState &stream_state, const Record *record,
                       const DoubleArray &filtered_data) {
  const auto n{static_cast<size_t>(filtered_data.size())};
  set_status(Status::kInProgress, 1);

  int start_idx{0};
  Core::Time start{record->timeWindow().startTime()};
  // check if processing start lies within the record
  if (!stream_state_.initialized) {
    start_idx =
        std::max(0, static_cast<int>(n) -
                        static_cast<int>(stream_state_.received_samples -
                                         stream_state_.needed_samples));
    const auto t{static_cast<double>(start_idx) / n};
    start =
        record->startTime() + Core::TimeSpan{record->timeWindow().length() * t};
  }

  double coefficient{std::nan("")};
  size_t lag_idx{0};
  // determine the first maximum correlation coefficient
  for (size_t i{static_cast<size_t>(start_idx)}; i < n; ++i) {
    const double v{filtered_data[i]};
    if (!isfinite(coefficient) || coefficient < v) {
      coefficient = v;
      lag_idx = i;
    }
  }

  // take cross-correlation filter delay into account i.e. the template
  // processor's result is referring to a time window shifted to the past
  const auto match_idx{
      static_cast<int>(lag_idx - cross_correlation_.template_size() + 1)};
  const auto t{static_cast<double>(match_idx) / n};
  const Core::TimeSpan template_length{cross_correlation_.template_length()};
  const Core::TimeWindow tw{start, record->endTime()};

  auto result{utils::make_smart<MatchResult>()};
  result->coefficient = coefficient;
  result->lag = tw.length() * t;
  result->time_window = tw;

  EmitResult(record, result.get());
}

void Template::Fill(StreamState &stream_state, const Record *record,
                    DoubleArrayPtr &data) {
  WaveformProcessor::Fill(stream_state, record, data);
  // cross-correlate filtered data
  cross_correlation_.Apply(data->size(), data->typedData());
}

void Template::SetupStream(StreamState &stream_state, const Record *record) {
  WaveformProcessor::SetupStream(stream_state, record);
  const auto f{stream_state.sampling_frequency};
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Initialize stream: sampling_frequency=%f",
                               f);
  if (target_sampling_frequency_ && target_sampling_frequency_ != f) {
    SCDETECT_LOG_DEBUG_PROCESSOR(this,
                                 "Reinitialize stream: sampling_frequency=%f",
                                 target_sampling_frequency_);
    auto resampling_operator{
        utils::make_unique<waveform_operator::ResamplingOperator>(
            RecordResamplerStore::Instance().Get(record,
                                                 *target_sampling_frequency_))};
    set_operator(resampling_operator.release());

    stream_state.sampling_frequency = *target_sampling_frequency_;
    if (stream_state.filter) {
      stream_state.filter->setSamplingFrequency(*target_sampling_frequency_);
    }
  }

  cross_correlation_.set_sampling_frequency(
      target_sampling_frequency_.value_or(f));
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
