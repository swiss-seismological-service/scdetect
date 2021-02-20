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

  Template(const std::string &template_id, const Processor *p = nullptr);

public:
  friend class TemplateBuilder;
  static TemplateBuilder Create(const std::string &template_id,
                                const Processor *p = nullptr);

  DEFINE_SMARTPOINTER(MatchResult);
  struct MatchResult : public Result {

    MatchResult(const double sum_template, const double squared_sum_template,
                const int num_samples_template, const Core::TimeWindow &tw);

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

    struct DebugInfo {
      std::string path_template;
      std::string path_trace;
    } debug_info;

    friend std::ostream &operator<<(std::ostream &os,
                                    const MatchResult &result);
  };

  const std::string id() const override;

  void set_filter(Filter *filter) override;

  const Core::TimeSpan init_time() const override;

  bool Feed(const Record *record) override;
  void Reset() override;

protected:
  void Process(StreamState &stream_state, const Record *record,
               const DoubleArray &filtered_data) override;

  void Fill(StreamState &stream_state, const Record *record,
            DoubleArrayPtr &data) override;

  void InitStream(StreamState &stream_state, const Record *record) override;

private:
  StreamState stream_state_;
  Processing::Stream stream_config_;

  // Template waveform starttime
  Core::Time waveform_start_;
  // Template waveform endtime
  Core::Time waveform_end_;
  // Template waveform
  Core::TimeSpan waveform_length_;
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

  const Processor *detector_{nullptr};
};

class TemplateBuilder : public Builder<Template> {
public:
  TemplateBuilder(const std::string &template_id, const Processor *p);
  TemplateBuilder &set_stream_config(const DataModel::Stream &stream_config);
  TemplateBuilder &
  // Set the template waveform of the `Template` waveform processor built.
  // While `wf_start` and `wf_end` refer to the target template waveform start
  // and end times, `wf_start_waveform` and `wf_end_waveform` refer to the
  // actual times of the resulting waveform.
  set_waveform(WaveformHandlerIfacePtr waveform_handler,
               const std::string &stream_id, const Core::Time &wf_start,
               const Core::Time &wf_end,
               const WaveformHandlerIface::ProcessingConfig &config,
               Core::Time &wf_start_waveform, Core::Time &wf_end_waveform);
  TemplateBuilder &set_filter(Processor::Filter *filter,
                              const double init_time = 0);
  TemplateBuilder &set_sensitivity_correction(bool enabled, double thres = -1);

  // Set the path to the debug info directory
  TemplateBuilder &set_debug_info_dir(const boost::filesystem::path &path);
};

namespace template_detail {

/* Calculate the maximum correlation coefficient and corresponding lag from
 * series `tr1` and a series `tr2` where `size_tr1` must be <= `size_tr2`
 * (i.e. `tr1` is cross-correlated with `tr2`).
 */
bool XCorr(const double *tr1, const int size_tr1, const double *tr2,
           const int size_tr2, const double sampling_freq,
           Template::MatchResultPtr &result,
           const Processor *processor = nullptr);

} // namespace template_detail

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_TEMPLATE_H_
