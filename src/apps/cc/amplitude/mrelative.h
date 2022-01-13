#ifndef SCDETECT_APPS_CC_AMPLITUDE_MRELATIVE_H_
#define SCDETECT_APPS_CC_AMPLITUDE_MRELATIVE_H_

#include <vector>

#include "../combining_amplitude_processor.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

class MRelative : public CombiningAmplitudeProcessor {
 public:
  static const CombiningStrategy median;

  explicit MRelative(
      std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying);

  void finalize(DataModel::Amplitude *amplitude) const override;

 private:
  static bool validateUniqueSensorLocation(
      const std::vector<std::string> &waveformStreamIDs);
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_MRELATIVE_H_
