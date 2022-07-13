#ifndef SCDETECT_APPS_CC_AMPLITUDE_RMS_H_
#define SCDETECT_APPS_CC_AMPLITUDE_RMS_H_

#include <seiscomp/core/timewindow.h>

#include <memory>

#include "../amplitude_processor.h"
#include "../processing/stream_config.h"
#include "seiscomp/core/datetime.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

class RMSAmplitude : public AmplitudeProcessor {
 public:
  enum class SignalUnit {
    // displacement
    kMeter = -1,
    // velocity
    kMeterPerSeconds,
    // acceleration
    kMeterPerSecondsSquared,
  };

  static SignalUnit signalUnitFromString(const std::string &signalUnit);

  void reset() override;
  // Sets the filter including the filter initialization time
  //
  // - implicitly resets the waveform processor
  void setFilter(std::unique_ptr<DoubleFilter> filter,
                 Core::TimeSpan initTime = Core::TimeSpan{0.0});

  void setDeconvolutionConfig(const DeconvolutionConfig &config);
  const DeconvolutionConfig &deconvolutionConfig() const;

  void setStreamConfig(const processing::StreamConfig &streamConfig);
  const processing::StreamConfig &streamConfig() const;

 protected:
  StreamState *streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  void preprocessData(StreamState &streamState,
                      const processing::StreamConfig &streamConfig,
                      const DeconvolutionConfig &deconvolutionConfig,
                      DoubleArray &data) override;

 private:
  AmplitudeProcessor::IndexRange computeIndexRange(
      const Core::TimeWindow &tw) const;

  processing::WaveformProcessor::StreamState _streamState;
  processing::StreamConfig _streamConfig;
  DeconvolutionConfig _deconvolutionConfig;

  Buffer _buffer;

  Core::TimeWindow _bufferedTimeWindow;
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_RMS_H_
