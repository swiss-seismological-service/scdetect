#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATEWAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATEWAVEFORMPROCESSOR_H_

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <boost/optional.hpp>
#include <cstdlib>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "../filter/crosscorrelation.h"
#include "../processing/waveform_processor.h"
#include "../template_waveform.h"

namespace Seiscomp {
namespace detect {
namespace detector {

namespace detail {

struct LocalMaxima {
  struct Value {
    double coefficient;
    size_t lagIdx;
  };

  using Values = std::vector<Value>;
  Values values;

  double prevCoefficient{-1};
  bool notDecreasing{false};

  void feed(double coefficient, std::size_t lagIdx);
};

}  // namespace detail

// Template waveform processor implementation
//
// - implements resampling and filtering
// - applies the cross-correlation algorithm
class TemplateWaveformProcessor : public processing::WaveformProcessor {
 public:
  // Creates a `TemplateWaveformProcessor`. Waveform related parameters are
  // forwarded to the underlying cross-correlation instance.
  explicit TemplateWaveformProcessor(TemplateWaveform templateWaveform);

  DEFINE_SMARTPOINTER(MatchResult);
  struct MatchResult : public Core::BaseObject {
    struct Value {
      Core::TimeSpan lag;
      double coefficient;
    };

    using LocalMaxima = std::vector<Value>;
    LocalMaxima localMaxima;

    // Time window for w.r.t. the match results
    Core::TimeWindow timeWindow;
  };
  using PublishMatchResultCallback = std::function<void(
      const TemplateWaveformProcessor *, const Record *, MatchResultCPtr)>;

  // Sets `filter` with the corresponding filter `initTime`
  void setFilter(std::unique_ptr<Filter> filter,
                 const Core::TimeSpan &initTime = Core::TimeSpan{0.0});
  // Returns the configured filter or `nullptr` if no filter has been configured
  const Filter *filter() const;

  // Sets the `callback` in order to publish detections
  void setResultCallback(const PublishMatchResultCallback &callback);

  // Returns the time window processed and correlated
  const Core::TimeWindow &processed() const;

  void reset() override;

  void setTargetSamplingFrequency(double f);
  boost::optional<double> targetSamplingFrequency() const;

  // Returns the underlying template waveform
  const TemplateWaveform &templateWaveform() const;

 protected:
  WaveformProcessor::StreamState *streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  void setupStream(StreamState &streamState, const Record *record) override;

  void emitResult(const Record *record, const MatchResultCPtr &result);

 private:
  StreamState _streamState;

  PublishMatchResultCallback _resultCallback;

  // The optional target sampling frequency (used for on-the-fly resampling)
  boost::optional<double> _targetSamplingFrequency;
  // The in-place cross-correlation filter
  filter::CrossCorrelation<double> _crossCorrelation;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_TEMPLATEWAVEFORMPROCESSOR_H_
