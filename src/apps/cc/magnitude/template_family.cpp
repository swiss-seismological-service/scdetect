#include "template_family.h"

#include <cassert>

namespace Seiscomp {
namespace detect {
namespace magnitude {

void TemplateFamilyBased::addAmplitudeMagnitude(
    DataModel::AmplitudeCPtr amplitude,
    DataModel::StationMagnitudeCPtr magnitude) {
  assert((amplitude && magnitude));
  _amplitudeMagnitudes.push_back({std::move(amplitude), std::move(magnitude)});
}

void TemplateFamilyBased::resetAmplitudeMagnitudes() {
  _amplitudeMagnitudes.clear();
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
