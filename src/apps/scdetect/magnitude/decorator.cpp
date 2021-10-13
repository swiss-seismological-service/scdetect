#include "decorator.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

Decorator::Decorator(MagnitudeProcessor* processor, const std::string& id)
    : MagnitudeProcessor{id}, _processor{processor} {}

double Decorator::compute(const DataModel::Amplitude* amplitude) {
  return _processor->compute(amplitude);
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
