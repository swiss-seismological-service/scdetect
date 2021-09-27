#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATEWAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATEWAVEFORMPROCESSOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <boost/optional.hpp>
#include <cstdlib>
#include <ostream>
#include <string>

#include "../filter/crosscorrelation.h"
#include "../waveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Template waveform processor implementation
// - implements resampling and filtering
// - applies the cross-correlation algorithm
class TemplateWaveformProcessor : public WaveformProcessor {
 public:
  // Creates a `TemplateWaveformProcessor`. Waveform related parameters are
  // forwarded to the underlying cross-correlation instance.
  TemplateWaveformProcessor(const GenericRecordCPtr &waveform,
                            const std::string filterId,
                            const Core::Time &templateStartTime,
                            const Core::Time &templateEndTime,
                            const std::string &processorId,
                            const Processor *p = nullptr);

  DEFINE_SMARTPOINTER(MatchResult);
  struct MatchResult : public Result {
    double coefficient{std::nan("")};
    double lag{};  // seconds

    // Time window for w.r.t. the match result
    Core::TimeWindow timeWindow;
  };

  void setFilter(Filter *filter, const Core::TimeSpan &initTime = 0.0) override;

  // Returns the time window processed and correlated
  const Core::TimeWindow &processed() const;

  void reset() override;

  void setTargetSamplingFrequency(double f);
  boost::optional<double> targetSamplingFrequency() const;

  // Returns the template waveform starttime
  boost::optional<const Core::Time> templateStartTime() const;
  // Returns the template waveform endtime
  boost::optional<const Core::Time> templateEndTime() const;
  // Returns the template waveform duration
  Core::TimeSpan templateDuration() const;

 protected:
  WaveformProcessor::StreamState &streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool fill(detect::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  void setupStream(StreamState &streamState, const Record *record) override;

 private:
  StreamState _streamState;

  // The optional target sampling frequency (used for on-the-fly resampling)
  boost::optional<double> _targetSamplingFrequency;
  // The in-place cross-correlation filter
  filter::AdaptiveCrossCorrelation<double> _crossCorrelation;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATEWAVEFORMPROCESSOR_H_
