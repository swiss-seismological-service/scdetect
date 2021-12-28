#ifndef SCDETECT_APPS_SCDETECT_AMPLITUDE_RATIO_H_
#define SCDETECT_APPS_SCDETECT_AMPLITUDE_RATIO_H_

#include <seiscomp/core/genericrecord.h>

#include "../amplitude_processor.h"
#include "../template_waveform.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

// Compute the amplitude ratio between the template waveform and the detected
// event
class Ratio : public AmplitudeProcessor {
 public:
  Ratio() = default;
  explicit Ratio(TemplateWaveform templateWaveform);

  // Computes time window based on the template waveform and the environment
  // pick time
  void computeTimeWindow() override;

  void setTemplateWaveform(TemplateWaveform templateWaveform);

 protected:
  StreamState &streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

 private:
  void initTemplateWaveform();

  TemplateWaveform _templateWaveform;

  processing::WaveformProcessor::StreamState _streamState;

  Buffer _buffer;
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_AMPLITUDE_RATIO_H_
