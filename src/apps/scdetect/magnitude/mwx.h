#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_MWX_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_MWX_H_

#include "regression.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class MWx : public FixedSlopeRegressionMagnitude<2, 3> {
 public:
  MWx();
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_MWX_H_
