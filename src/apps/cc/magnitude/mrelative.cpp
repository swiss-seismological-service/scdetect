#include "mrelative.h"

#include <cassert>
#include <cmath>

#include "seiscomp/datamodel/databasearchive.h"
#include "seiscomp/datamodel/stationmagnitude.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

MRelative::MRelative(TemplateMagnitude templateMagnitude)
    : _templateMagnitude{templateMagnitude} {
  setType("MRelative");
}

void MRelative::finalize(DataModel::StationMagnitude* magnitude) const {
  magnitude->setType(type());
}

void MRelative::setTemplateMagnitude(TemplateMagnitude templateMagnitude) {
  _templateMagnitude = templateMagnitude;
}

const MRelative::TemplateMagnitude& MRelative::templateMagnitude() const {
  return _templateMagnitude;
}

double MRelative::computeMagnitude(const DataModel::Amplitude* amplitude) {
  assert(amplitude);

  const auto amplitudeValue{amplitude->amplitude().value()};
  assert((std::isfinite(amplitudeValue) && amplitudeValue > 0));

  return _templateMagnitude.value + std::log10(amplitudeValue);
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
