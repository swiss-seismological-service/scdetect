#include "mrelative.h"

#include <memory>
#include <vector>

#include "../util/memory.h"
#include "factory.h"
#include "ratio.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

const MRelative::CombiningStrategy MRelative::median =
    [](const std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> &amplitudes,
       detect::AmplitudeProcessor::Amplitude &combined) {
      assert(!amplitudes.empty());
      std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> sorted(
          static_cast<std::size_t>(
              std::ceil(static_cast<double>(amplitudes.size()) / 2)));
      std::partial_sort_copy(
          std::begin(amplitudes), std::end(amplitudes), std::begin(sorted),
          std::end(sorted),
          [](const detect::AmplitudeProcessor::AmplitudeCPtr &lhs,
             const detect::AmplitudeProcessor::AmplitudeCPtr &rhs) {
            return lhs->value.value < rhs->value.value;
          });

      combined = *sorted.back();
    };

MRelative::MRelative(
    std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> &&underlying,
    CombiningStrategy strategy)
    : CombiningAmplitudeProcessor{std::move(underlying), std::move(strategy)} {
  setType("MRelative");
  setUnit("");
}

void MRelative::finalize(DataModel::Amplitude *amplitude) const {
  CombiningAmplitudeProcessor::finalize(amplitude);

  amplitude->setType(type());
  amplitude->setUnit(unit());
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
