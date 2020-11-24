#ifndef SCDETECT_APPS_SCDETECT_SETTINGS_H_
#define SCDETECT_APPS_SCDETECT_SETTINGS_H_

#include <string>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace settings {

// Relative path from the SeisComP installation directory
const std::string kPathFilesystemCache{"var/cache/scdetect"};

// Template specific default configuration
const std::vector<std::string> kValidPhases{"Pg", "P", "Px", "Sg", "S", "Sx"};
const std::string kMagnitudeType{"SCDETECT"};
const std::string kOriginMethod{kMagnitudeType};
const std::string kStationMagnitudeType{kMagnitudeType};

constexpr double kBufferMultiplicator{2};

// The processing interval in seconds
/* constexpr double kDefaultProcessingInterval{0}; */

} // namespace settings
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_SETTINGS_H_
