#include "decorator.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

Decorator::Decorator(std::unique_ptr<MagnitudeProcessor>&& decorated)
    : _decorated{std::move(decorated)} {}

double Decorator::compute(const DataModel::Amplitude* amplitude) {
  return _decorated->compute(amplitude);
}

void Decorator::finalize(DataModel::StationMagnitude* magnitude) const {
  _decorated->finalize(magnitude);
}

MagnitudeProcessor* Decorator::decorated() { return _decorated.get(); }

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
