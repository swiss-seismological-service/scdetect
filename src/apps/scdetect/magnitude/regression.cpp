#include "regression.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

MwxFixedSlopeRegressionMagnitude::MwxFixedSlopeRegressionMagnitude() {
  setType("Mwx");
}

MLxFixedSlopeRegressionMagnitude::MLxFixedSlopeRegressionMagnitude() {
  setType("MLx");
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
