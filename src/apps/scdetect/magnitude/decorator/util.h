#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_UTIL_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_UTIL_H_

#include <seiscomp/datamodel/amplitude.h>

#include <boost/optional/optional.hpp>
#include <string>

namespace Seiscomp {
namespace detect {
namespace magnitude {
namespace decorator {

// Extract the detector identifier from `amplitude`
boost::optional<std::string> extractDetectorId(
    const DataModel::Amplitude* amplitude);

// Extract the sensor location identifier from `amplitude`
boost::optional<std::string> extractSensorLocationId(
    const DataModel::Amplitude* amplitude);

}  // namespace decorator
}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_UTIL_H_
