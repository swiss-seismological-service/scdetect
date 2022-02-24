#include "mrelative.h"

#include <cassert>
#include <cmath>

#include "seiscomp/datamodel/databasearchive.h"
#include "seiscomp/datamodel/stationmagnitude.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

MRelative::MRelative(DataModel::StationMagnitudeCPtr templateMagnitude)
    : _templateMagnitude{std::move(templateMagnitude)} {
  setType("MRelative");
}

void MRelative::finalize(DataModel::StationMagnitude* magnitude) const {
  magnitude->setType(type());
}

void MRelative::setTemplateMagnitude(
    DataModel::StationMagnitudeCPtr templateMagnitude) {
  _templateMagnitude = std::move(templateMagnitude);
}

double MRelative::computeMagnitude(const DataModel::Amplitude* amplitude) {
  assert(amplitude);
  assert(_templateMagnitude);

  const auto amplitudeValue{amplitude->amplitude().value()};
  assert((std::isfinite(amplitudeValue) && amplitudeValue > 0));

  return _templateMagnitude->magnitude().value() + std::log10(amplitudeValue);
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
