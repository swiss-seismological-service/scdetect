#include "regression.h"

#include <cfenv>
#include <cmath>
#include <vector>

#include "../log.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

RegressionMagnitude::RegressionMagnitude(const std::string& id)
    : MagnitudeProcessor{id} {}

void RegressionMagnitude::add(
    const std::vector<RegressionMagnitude::AmplitudeMagnitude>&
        amplitudeMagnitudes) {
  _amplitudeMagnitudes.insert(std::end(_amplitudeMagnitudes),
                              std::begin(amplitudeMagnitudes),
                              std::end(amplitudeMagnitudes));
}

void RegressionMagnitude::add(
    const RegressionMagnitude::AmplitudeMagnitude& amplitudeMagnitude) {
  _amplitudeMagnitudes.push_back(amplitudeMagnitude);
}

void RegressionMagnitude::reset() { _amplitudeMagnitudes.clear(); }

/* ------------------------------------------------------------------------- */
MwFixedSlopeRegressionMagnitude::MwFixedSlopeRegressionMagnitude(
    const std::string& id)
    : FixedSlopeRegressionMagnitude{id} {
  _type = "Mw";
}

MLFixedSlopeRegressionMagnitude::MLFixedSlopeRegressionMagnitude(
    const std::string& id)
    : FixedSlopeRegressionMagnitude{id} {
  _type = "ML";
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
