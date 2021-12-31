#ifndef SCDETECT_APPS_SCDETECT_AMPLITUDE_MRELATIVE_H_
#define SCDETECT_APPS_SCDETECT_AMPLITUDE_MRELATIVE_H_

#include <vector>

#include "../combining_amplitude_processor.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

class MRelative : public CombiningAmplitudeProcessor {
 public:
  static const CombiningStrategy median;

  explicit MRelative(
      std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> &&underlying,
      CombiningStrategy strategy = median);

  void finalize(DataModel::Amplitude *amplitude) const override;
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_AMPLITUDE_MRELATIVE_H_
