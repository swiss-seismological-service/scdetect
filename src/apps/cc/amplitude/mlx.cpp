#include "mlx.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

MLx::MLx() {
  setType("MLx");
  setUnit("M/S");
}

void MLx::finalize(DataModel::Amplitude *amplitude) const {
  RMSAmplitude::finalize(amplitude);

  amplitude->setType(type());
  amplitude->setUnit(unit());
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
