#ifndef SCDETECT_APPS_CC_AMPLITUDE_RMS_H_
#define SCDETECT_APPS_CC_AMPLITUDE_RMS_H_

#include <memory>

#include "../amplitude_processor.h"
#include "../template_waveform.h"

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

  RMSAmplitude() = default;
  explicit RMSAmplitude(const TemplateWaveform &templateWaveform);

  static SignalUnit signalUnitFromString(const std::string &signalUnit);

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
  const TemplateWaveform &templateWaveform() const;

  void setDeconvolutionConfig(const DeconvolutionConfig &config);
  const DeconvolutionConfig &deconvolutionConfig() const;

  void setStreamConfig(const StreamConfig &streamConfig);
  const StreamConfig &streamConfig() const;

 protected:
  StreamState *streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  void preprocessData(StreamState &streamState,
                      const StreamConfig &streamConfig,
                      const DeconvolutionConfig &deconvolutionConfig,
                      DoubleArray &data) override;

 private:
  // Computes the new time window based on the template waveform related
  // `startTime` and `endTime` relative to the configured pick time
  Core::TimeWindow computeTimeWindow(const Core::Time &startTime,
                                     const Core::Time &endTime) const;

  AmplitudeProcessor::IndexRange computeIndexRange(
      const Core::TimeWindow &tw) const;

  TemplateWaveform _templateWaveform;

  processing::WaveformProcessor::StreamState _streamState;
  StreamConfig _streamConfig;
  DeconvolutionConfig _deconvolutionConfig;

  Buffer _buffer;

  Core::TimeWindow _bufferedTimeWindow;
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_RMS_H_
