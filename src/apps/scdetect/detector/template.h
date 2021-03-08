#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATE_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATE_H_

#include <cstdlib>
#include <ostream>
#include <string>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/stream.h>

#include "../filter/crosscorrelation.h"
#include "../waveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Template waveform processor implementation
// - implements filtering
// - applies the cross-correlation algorithm
class Template : public WaveformProcessor {

public:
  // Creates a `Template` waveform processor from the template waveform
  // `template_wf`
  Template(const GenericRecordCPtr &template_wf, const std::string &id,
           const Processor *p = nullptr);

  DEFINE_SMARTPOINTER(MatchResult);
  struct MatchResult : public Result {
    double coefficient{std::nan("")};
    double lag{}; // seconds

    // Time window for w.r.t. the match result
    Core::TimeWindow time_window;
  };

  void set_filter(Filter *filter) override;

  const Core::TimeWindow &processed() const override;

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

  // The in-place cross-correlation filter
  filter::CrossCorrelation<double> cross_correlation_;
};

} // namespace detector
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATE_H_
