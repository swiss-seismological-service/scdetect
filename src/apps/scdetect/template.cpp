#include "template.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "processor.h"
#include "settings.h"
#include "utils.h"
#include "version.h"
#include "waveform.h"
#include "xcorr.h"

namespace Seiscomp {
namespace detect {

Template::MatchResult::MatchResult(const double squared_sum_template,
                                   MatchResult::MetaData metadata)
    : squared_sum_template(squared_sum_template), metadata(metadata) {}

Template::Template() : pick_(nullptr), phase_(""), waveform_(nullptr) {}

TemplateBuilder Template::Create() { return TemplateBuilder(); }

void Template::set_filter(Filter *filter) {
  if (stream_state_.filter)
    delete stream_state_.filter;

  stream_state_.filter = filter;
}

const Core::TimeSpan &Template::init_time() const {
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

  if (!tmp) {
    stream_state_.filter = tmp->clone();
    delete tmp;
  }

  Processor::Reset();
  data_ = DoubleArray();
}

void Template::Process(StreamState &stream_state, RecordCPtr record,
                       const DoubleArray &filtered_data) {
  MatchResultPtr result{new MatchResult{waveform_squared_sum_,
                                        MatchResult::MetaData{pick_, phase_}}};

  const double *samples_template{
      DoubleArray::ConstCast(waveform_->data())->typedData()};
  const double *samples_trace{filtered_data.typedData()};
  const int num_samples_template{waveform_->data()->size()};
  const int num_samples_trace{filtered_data.size()};

  if (num_samples_template > num_samples_trace) {
    set_status(Status::kWaitingForData,
               num_samples_template - num_samples_trace);
    return;
  }

  set_status(Status::kInProgress, 1);
  // compute as much correlations as possible
  const int max_lag_samples{num_samples_trace - num_samples_template -
                            (num_samples_trace - num_samples_template) / 2};

  // TODO(damb): Make sure that the mean value is removed from both the
  // template waveform and filtered_data. Most probably this is done easiest
  // when validating the filter string.
  if (xcorr(samples_template, num_samples_template, samples_trace,
            num_samples_trace, waveform_sampling_frequency_, max_lag_samples,
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

  // resample
  if (waveform_sampling_frequency_ != stream_state_.sampling_frequency) {
    if (waveform_sampling_frequency_ < stream_state.sampling_frequency) {
      DoubleArrayPtr tmp{new DoubleArray{static_cast<int>(n), samples}};
      waveform::Resample(tmp, stream_state.sampling_frequency,
                         waveform_sampling_frequency_, true);

      data_.append(tmp->size(), tmp->typedData());
    } else {
      GenericRecordPtr resampled{new GenericRecord{*waveform_}};
      waveform::Resample(*resampled, stream_state.sampling_frequency, true);
      waveform_ = resampled;
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
TemplateBuilder::TemplateBuilder()
    : template_(std::unique_ptr<Template>(new Template{})) {}

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
  template_->waveform_ =
      waveform_handler->Get(stream_id, wf_start, wf_end, config);
  template_->waveform_sampling_frequency_ =
      template_->waveform_->samplingFrequency();

  const double *samples_template{
      DoubleArray::ConstCast(template_->waveform_->data())->typedData()};
  for (int i = 0; i < template_->waveform_->data()->size(); ++i) {
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

TemplateBuilder::operator std::unique_ptr<Processor>() {
  return std::move(template_);
}

} // namespace detect
} // namespace Seiscomp
