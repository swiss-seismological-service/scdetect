#ifndef SCDETECT_APPS_CC_MAGNITUDE_MLX_H_
#define SCDETECT_APPS_CC_MAGNITUDE_MLX_H_

#include "regression.h"
#include "seiscomp/datamodel/stationmagnitude.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class MLx : public FixedSlopeRegressionMagnitude<1> {
 public:
  MLx();
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_MAGNITUDE_MLX_H_
