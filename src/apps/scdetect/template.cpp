#include "template.h"

#include <boost/algorithm/string/join.hpp>

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include "processor.h"
#include "settings.h"
#include "utils.h"
#include "version.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

Template::MatchResult::MatchResult(const double sum_template,
                                   const double squared_sum_template,
                                   const int num_samples_template,
                                   MatchResult::MetaData metadata)
    : num_samples_template{num_samples_template}, sum_template{sum_template},
      squared_sum_template{squared_sum_template}, metadata(metadata) {}

Template::Template() : pick_{nullptr}, phase_{""}, waveform_{nullptr} {
  Reset();
}

TemplateBuilder Template::Create() { return TemplateBuilder(); }

void Template::set_filter(Filter *filter) {
  if (stream_state_.filter)
    delete stream_state_.filter;

  stream_state_.filter = filter;
}

const Core::TimeSpan Template::init_time() const {
  return std::max(init_time_, waveform_end_ - waveform_start_);
}

bool Template::Feed(const Record *record) {
  if (record->sampleCount() == 0)
    return false;

  return Store(stream_state_, record);
}

void Template::Reset() {
  Filter *tmp{stream_state_.filter};

  stream_state_ = StreamState{};
  if (tmp) {
    stream_state_.filter = tmp->clone();
    delete tmp;
  }

  Processor::Reset();
  data_ = DoubleArray();
}

void Template::Process(StreamState &stream_state, RecordCPtr record,
                       const DoubleArray &filtered_data) {
  const double *samples_template{
      DoubleArray::ConstCast(waveform_->data())->typedData()};
  const double *samples_trace{data_.typedData()};
  const int num_samples_template{waveform_->data()->size()};
  const int num_samples_trace{data_.size()};

  MatchResultPtr result{utils::make_smart<MatchResult>(
      waveform_sum_, waveform_squared_sum_, num_samples_template,
      MatchResult::MetaData{pick_, phase_})};

  if (num_samples_template > num_samples_trace) {
    set_status(Status::kWaitingForData,
               num_samples_template - num_samples_trace);
    return;
  }

  set_status(Status::kInProgress, 1);
  // compute as much correlations as possible
  const int max_lag_samples{num_samples_trace - num_samples_template -
                            (num_samples_trace - num_samples_template) / 2};

  if (template_detail::XCorr(samples_template, num_samples_template,
                             samples_trace, num_samples_trace,
                             waveform_sampling_frequency_, max_lag_samples,
                             result)) {
    EmitResult(record, result);
    set_status(Processor::Status::kFinished, 100);
    return;
  }
  set_status(Processor::Status::kError, 0);
}

void Template::Fill(StreamState &stream_state, RecordCPtr record, size_t n,
                    double *samples) {
  Processor::Fill(stream_state, record, n, samples);

  // demean
  double mean{0};
  for (size_t i = 0; i < n; ++i) {
    mean += samples[i];
  }
  mean /= n;

  for (size_t i = 0; i < n; ++i) {
    samples[i] -= mean;
  }

  // resample (i.e. always downsample)
  if (waveform_sampling_frequency_ != stream_state_.sampling_frequency) {
    if (waveform_sampling_frequency_ < stream_state.sampling_frequency) {
      auto tmp{utils::make_smart<DoubleArray>(static_cast<int>(n), samples)};
      waveform::Resample(tmp, stream_state.sampling_frequency,
                         waveform_sampling_frequency_, true);

      n = tmp->size();
      samples = tmp->typedData();
    } else {
      auto resampled{utils::make_smart<GenericRecord>(*waveform_)};
      waveform::Resample(*resampled, stream_state.sampling_frequency, true);
      waveform_ = resampled;
      waveform_sampling_frequency_ = stream_state.sampling_frequency;
    }
  }
  data_.append(n, samples);
}

void Template::InitFilter(StreamState &stream_state, double sampling_freq) {
  Processor::InitFilter(stream_state, sampling_freq);

  stream_state.needed_samples =
      std::max(static_cast<size_t>(waveform_->sampleCount()),
               stream_state.needed_samples);
}

/* ------------------------------------------------------------------------- */
// XXX(damb): Using `new` to access a non-public ctor; see also
// https://abseil.io/tips/134
TemplateBuilder::TemplateBuilder() : template_(new Template{}) {}

TemplateBuilder &
TemplateBuilder::set_stream_config(const DataModel::Stream &stream_config) {
  template_->stream_config_.init(&stream_config);
  return *this;
}

TemplateBuilder &TemplateBuilder::set_phase(const std::string &phase) {
  template_->phase_ = phase;
  return *this;
}

TemplateBuilder &TemplateBuilder::set_pick(DataModel::PickCPtr pick) {
  template_->pick_ = pick;
  return *this;
}

TemplateBuilder &TemplateBuilder::set_arrival_weight(const double weight) {
  template_->arrival_weight_ = weight;
  return *this;
}

TemplateBuilder &TemplateBuilder::set_waveform(
    WaveformHandlerIfacePtr waveform_handler, const std::string &stream_id,
    const Core::Time &wf_start, const Core::Time &wf_end,
    const WaveformHandlerIface::ProcessingConfig &config) {

  template_->waveform_start_ = wf_start;
  template_->waveform_end_ = wf_end;
  template_->waveform_stream_id_ = stream_id;

  // prepare waveform stream id
  std::vector<std::string> wf_tokens;
  Core::split(wf_tokens, stream_id, ".", false);
  try {
    template_->waveform_ =
        waveform_handler->Get(wf_tokens[0], wf_tokens[1], wf_tokens[2],
                              wf_tokens[3], wf_start, wf_end, config);
  } catch (WaveformHandler::NoData &e) {
    throw builder::NoWaveformData{
        std::string{"Failed to load template waveform: "} + e.what()};
  } catch (std::exception &e) {
    throw builder::BaseException{
        std::string{"Failed to load template waveform: "} + e.what()};
  }
  template_->waveform_sampling_frequency_ =
      template_->waveform_->samplingFrequency();

  const double *samples_template{
      DoubleArray::ConstCast(template_->waveform_->data())->typedData()};
  for (int i = 0; i < template_->waveform_->data()->size(); ++i) {
    template_->waveform_sum_ += samples_template[i];
    template_->waveform_squared_sum_ +=
        samples_template[i] * samples_template[i];
  }
  return *this;
}

TemplateBuilder &TemplateBuilder::set_publish_callback(
    const Processor::PublishResultCallback &callback) {
  template_->set_result_callback(callback);
  return *this;
}

TemplateBuilder &TemplateBuilder::set_filter(Processor::Filter *filter,
                                             const double init_time) {
  template_->set_filter(filter);
  template_->init_time_ = Core::TimeSpan{init_time};
  return *this;
}

TemplateBuilder &TemplateBuilder::set_sensitivity_correction(bool enabled,
                                                             double thres) {
  template_->set_saturation_check(enabled);
  template_->set_saturation_threshold(thres);
  return *this;
}

ProcessorPtr TemplateBuilder::build() { return template_; }

/* ------------------------------------------------------------------------- */
namespace template_detail {

bool XCorr(const double *tr1, const int size_tr1, const double *tr2,
           const int size_tr2, const double sampling_freq,
           const double max_lag_samples, Template::MatchResultPtr result) {

  /*
   * Pearson correlation coefficient for time series X and Y of length n
   *
   *              sum((Xi-meanX) * (Yi-meanY))
   * cc = --------------------------------------------------
   *      sqrt(sum((Xi-meanX)^2)) * sqrt(sum((Yi-meanY)^2))
   *
   * Where sum(X)  is the sum of Xi for i=1 until i=n
   *
   * This can be rearranged in a form suitable for a single-pass algorithm
   * (where the mean of X and Y are not needed)
   *
   *                 n * sum(Xi*Yi) - sum(Xi) * sum(Yi)
   * cc = -----------------------------------------------------------
   *      sqrt(n*sum(Xi^2)-sum(Xi)^2) * sqrt(n*sum(Yi^2)-sum(Yi)^2))
   *
   * For cross-correlation, where we have a short trace `tr1` which is
   * correlated against a longer trace `tr2` at subsequent offset, we can
   * pre-compute the parts that involve `tr1` and re-use them at each step of
   * the cross-correlation:
   *
   *   sum_short = sum(Xi)
   *   squared_sum_short = sum(Xi^2)
   *   denominator_short = sqrt(n*squared_sum_short-(sum_short)^2)
   *
   * For the parts that involves the longer trace L alone we can compute them
   * in a rolling fashion (removing first sample of previous iteration and
   * adding the last sample of the new iteration):
   *
   *   sum_long = sum(Yi)
   *   squared_sum_long = sum(Yi^2)
   *   denominator_long = sqrt(n*squared_sum_long-(sum_long)^2)
   *
   * Finally, this is the equation at each step (offset) of cross-correlation:
   *
   *       n * sum(Xi*Yi) - sum_short  * sum_long
   * cc = ---------------------------------------
   *        denominator_short * denominator_long
   *
   * Unfortunately, we cannot optimize sum(Xi*Yi) and this will be a inner
   * loop inside the main cross-correlation loop
   */

  auto SampleAtLong = [&size_tr1, &size_tr2, &tr2](int idx_short, int lag) {
    // TODO(damb): To be clarified with luca-s.
    /* const int idx_long{idx_short + (size_tr2 - size_tr1) / 2 + lag}; */
    const int idx_long{idx_short + lag};
    return (idx_long < 0 || idx_long >= size_tr2) ? 0 : tr2[idx_long];
  };

  if (size_tr2 < size_tr1) {
    return false;
  }

  if (!result->squared_sum_template)
    return false;

  result->coefficient = std::nan("");
  std::feclearexcept(FE_ALL_EXCEPT);

  // do as much computation as possible outside the main cross-correlation loop
  const auto &n = size_tr1;
  const auto &sum_short = result->sum_template;
  const auto &squared_sum_short = result->squared_sum_template;
  const double denominator_short{
      std::sqrt(n * squared_sum_short - sum_short * sum_short)};

  if (!std::isfinite(denominator_short)) {
    std::feclearexcept(FE_INVALID);
    return false;
  }

  double sum_long{0};
  double squared_sum_long{0};
  for (int i = 0; i < n; ++i) {
    double sample{SampleAtLong(i, -(max_lag_samples + 1))};
    sum_long += sample;
    squared_sum_long += sample * sample;
  }

  // cross-correlation loop
  for (int lag = -max_lag_samples; lag <= max_lag_samples; ++lag) {
    // sum_long/squared_sum_long: remove the sample that has exited the current
    // xcorr win and add the sample that has just entered the current xcorr win
    const double last_sample_long{SampleAtLong(-1, lag)};
    const double new_sample_long{SampleAtLong(n - 1, lag)};
    sum_long += new_sample_long - last_sample_long;
    squared_sum_long +=
        new_sample_long * new_sample_long - last_sample_long * last_sample_long;

    const double denominator_long{
        std::sqrt(n * squared_sum_long - sum_long * sum_long)};

    if (!std::isfinite(denominator_long)) {
      continue;
    }

    double sum_short_long{0};
    for (int i = 0; i < n; ++i) {
      sum_short_long += tr1[i] * SampleAtLong(i, lag);
    }

    const double coeff{(n * sum_short_long - sum_short * sum_long) /
                       (denominator_short * denominator_long)};

    if (!std::isfinite(result->coefficient) ||
        (std::isfinite(coeff) &&
         std::abs(coeff) > std::abs(result->coefficient))) {
      result->sum_trace = sum_long;
      result->squared_sum_trace = squared_sum_long;
      result->sum_template_trace = sum_short_long;
      result->coefficient = coeff;
      result->lag = lag / sampling_freq; // samples to seconds
    }
  }

  int fe{fetestexcept(FE_ALL_EXCEPT)};
  if ((fe & ~FE_INEXACT) != 0) // we don't care about FE_INEXACT
  {
    std::vector<std::string> exceptions;
    if (fe & FE_DIVBYZERO)
      exceptions.push_back("FE_DIVBYZERO");
    if (fe & FE_INVALID)
      exceptions.push_back("FE_INVALID");
    if (fe & FE_OVERFLOW)
      exceptions.push_back("FE_OVERFLOW");
    if (fe & FE_UNDERFLOW)
      exceptions.push_back("FE_UNDERFLOW");

    std::string msg{"Floating point exception during cross-correlation: "};
    msg += boost::algorithm::join(exceptions, ", ");
    SEISCOMP_WARNING(msg.c_str());
  }

  if (!std::isfinite(result->coefficient)) {
    result->coefficient = 0;
    result->lag = 0;
    result->sum_template_trace = 0;
    result->sum_template = 0;
    result->squared_sum_template = 0;

    return false;
  }

  return true;
}

} // namespace template_detail

} // namespace detect
} // namespace Seiscomp
