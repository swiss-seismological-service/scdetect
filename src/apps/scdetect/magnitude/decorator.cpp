#include "decorator.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

Decorator::Decorator(std::unique_ptr<MagnitudeProcessor>&& decorated)
    : _decorated{std::move(decorated)} {}

double Decorator::compute(const DataModel::Amplitude* amplitude) {
  return _decorated->compute(amplitude);
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
