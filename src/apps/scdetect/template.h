#ifndef SCDETECT_APPS_SCDETECT_TEMPLATE_H_
#define SCDETECT_APPS_SCDETECT_TEMPLATE_H_

#include <cstdlib>
#include <ostream>
#include <string>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
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

  Template(const std::string &template_id);

public:
  friend class TemplateBuilder;
  static TemplateBuilder Create(const std::string &template_id);

  DEFINE_SMARTPOINTER(MatchResult);
  struct MatchResult : public Result {

    struct MetaData;
    MatchResult(const double sum_template, const double squared_sum_template,
                const int num_samples_template, const Core::TimeWindow &tw,
                MetaData metadata);

    double coefficient{std::nan("")};
    int num_samples_template;
    double sum_template{};
    double squared_sum_template{};
    double sum_trace{};
    double squared_sum_trace{};
    double sum_template_trace{};
    double lag{}; // seconds

    // Time window for w.r.t. the match result
    Core::TimeWindow time_window;

    struct MetaData {
      // Original template pick
      DataModel::PickCPtr pick;
      // Template phase
      std::string phase;
      // Template arrival weight
      double arrival_weight;
    } metadata;

    struct DebugInfo {
      std::string path_template;
      std::string path_trace;
    } debug_info;

    friend std::ostream &operator<<(std::ostream &os,
                                    const MatchResult &result);
  };

  void set_filter(Filter *filter) override;
  const Core::TimeSpan init_time() const override;

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
  DataModel::PickCPtr pick_{nullptr};
  // Template related phase code
  std::string phase_{""};
  // Template related arrival weight
  double arrival_weight_{0};

  // Template waveform starttime
  Core::Time waveform_start_;
  // Template waveform endtime
  Core::Time waveform_end_;
  // Template waveform stream id
  std::string waveform_stream_id_;
  // Template waveform
  GenericRecordCPtr waveform_{nullptr};
  // Template waveform sampling frequency
  double waveform_sampling_frequency_{0};
  // Template waveform filter string
  std::string waveform_filter_;
  // Template waveform samples squared summed
  double waveform_squared_sum_{0};
  // Template waveform samples summed
  double waveform_sum_{0};
};

class TemplateBuilder : public Builder<TemplateBuilder> {
public:
  TemplateBuilder(const std::string &template_id);
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
  TemplateBuilder &set_sensitivity_correction(bool enabled, double thres = -1);

  // Set the path to the debug info directory
  TemplateBuilder &set_debug_info_dir(const boost::filesystem::path &path);

  ProcessorPtr build();

private:
  TemplatePtr template_;
};

namespace template_detail {

/* Calculate the maximum correlation coefficient and corresponding lag from
 * series `tr1` and a series `tr2` where `size_tr1` must be <= `size_tr2`
 * (i.e. `tr1` is cross-correlated with `tr2`).
 */
bool XCorr(const double *tr1, const int size_tr1, const double *tr2,
           const int size_tr2, const double sampling_freq,
           Template::MatchResultPtr result, ProcessorCPtr processor = nullptr);

} // namespace template_detail

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_TEMPLATE_H_
