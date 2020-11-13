#ifndef SCDETECT_APPS_SCDETECT_TEMPLATE_H_
#define SCDETECT_APPS_SCDETECT_TEMPLATE_H_

#include <memory>
#include <string>

#include <seiscomp/core/datetime.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/stream.h>

#include "builder.h"
#include "processor.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

class TemplateBuilder;

DEFINE_SMARTPOINTER(Template);
class Template : public Processor {

  Template();

public:
  friend class TemplateBuilder;
  static TemplateBuilder Create();

  DEFINE_SMARTPOINTER(MatchResult);
  struct MatchResult : public Result {

    struct MetaData;
    MatchResult(const double squared_sum_template, MetaData metadata);

    double coefficient{std::nan("")};
    double numerator{};
    double squared_sum_template{};
    double squared_sum_trace{};
    double lag{}; // secs

    struct MetaData {
      DataModel::PickCPtr pick;
      std::string phase;
      double arrival_weight;
    } metadata;
  };

  void set_filter(Filter *filter) override;
  const Core::TimeSpan &init_time() const override;

  bool Feed(const Record *record) override;
  void Reset() override;

protected:
  void Process(StreamState &stream_state, RecordCPtr record,
               const DoubleArray &filtered_data) override;

  void Fill(StreamState &stream_state, RecordCPtr record, size_t n,
            double *samples) override;

  void InitFilter(StreamState &stream_state, double sampling_freq) override;

  DoubleArray data_;

private:
  StreamState stream_state_;
  Processing::Stream stream_config_;

  // Template related pick
  DataModel::PickCPtr pick_;
  // Template related phase code
  std::string phase_;
  // Template related arrival weight
  double arrival_weight_;

  // Template waveform starttime
  Core::Time waveform_start_;
  // Template waveform endtime
  Core::Time waveform_end_;
  // Template waveform stream id
  std::string waveform_stream_id_;
  // Template waveform
  GenericRecordCPtr waveform_;
  // Template waveform sampling frequency
  double waveform_sampling_frequency_;
  // Template waveform samples squared summed
  double waveform_squared_sum_;
};

class TemplateBuilder : public Builder<TemplateBuilder> {
public:
  TemplateBuilder();
  TemplateBuilder &set_stream_config(const DataModel::Stream &stream_config);
  TemplateBuilder &set_phase(const std::string &phase);
  TemplateBuilder &set_pick(DataModel::PickCPtr pick);
  TemplateBuilder &set_arrival_weight(const double weight);
  TemplateBuilder &
  set_waveform(WaveformHandlerIfacePtr waveform_handler,
               const std::string &stream_id, const Core::Time &wf_start,
               const Core::Time &wf_end,
               const WaveformHandlerIface::ProcessingConfig &config);
  TemplateBuilder &
  set_publish_callback(const Processor::PublishResultCallback &callback);
  TemplateBuilder &set_filter(Processor::Filter *filter,
                              const double init_time = 0);

  operator std::unique_ptr<Processor>();

private:
  std::unique_ptr<Template> template_{nullptr};
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_TEMPLATE_H_
