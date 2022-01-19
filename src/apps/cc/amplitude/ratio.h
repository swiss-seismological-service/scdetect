#ifndef SCDETECT_APPS_CC_AMPLITUDE_RATIO_H_
#define SCDETECT_APPS_CC_AMPLITUDE_RATIO_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <cstddef>
#include <memory>

#include "../amplitude_processor.h"
#include "../def.h"
#include "../template_waveform.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

// Compute the amplitude ratio between the template waveform and the detected
// event
class RatioAmplitude : public AmplitudeProcessor {
 public:
  RatioAmplitude() = default;
  explicit RatioAmplitude(TemplateWaveform templateWaveform);

  void reset() override;
  // Computes time window based on the template waveform and the environment
  // pick time
  void computeTimeWindow() override;

  // Sets the filter including the filter initialization time
  //
  // - implicitly resets the waveform processor
  void setFilter(std::unique_ptr<DoubleFilter> filter,
                 Core::TimeSpan initTime = Core::TimeSpan{0.0});

  void setTemplateWaveform(const TemplateWaveform &templateWaveform);

 protected:
  StreamState *streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

 private:
  struct IndexRange {
    std::size_t begin;
    std::size_t end;
  };

  // Preprocesses the data and returns the signal time window
  void preprocess();

  // Computes the new time window based on the template waveform related
  // `startTime` and `endTime` relative to the configured pick time
  Core::TimeWindow computeTimeWindow(const Core::Time &startTime,
                                     const Core::Time &endTime) const;

  IndexRange computeIndexRange(const Core::TimeWindow &tw) const;

  void initTemplateWaveform();

  TemplateWaveform _templateWaveform;

  processing::WaveformProcessor::StreamState _streamState;

  Buffer _buffer;

  Core::TimeWindow _bufferedTimeWindow;
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_RATIO_H_
