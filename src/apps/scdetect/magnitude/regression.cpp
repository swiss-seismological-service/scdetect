#include "regression.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

MwxFixedSlopeRegressionMagnitude::MwxFixedSlopeRegressionMagnitude() {
  _type = "Mwx";
}

MLxFixedSlopeRegressionMagnitude::MLxFixedSlopeRegressionMagnitude() {
  _type = "MLx";
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
