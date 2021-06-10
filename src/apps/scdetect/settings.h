#ifndef SCDETECT_APPS_SCDETECT_SETTINGS_H_
#define SCDETECT_APPS_SCDETECT_SETTINGS_H_

#include <string>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace settings {

// Relative path from the SeisComP installation directory
const std::string kPathFilesystemCache{"var/cache/scdetect"};
// Relative path from the SeisComP installation directory
const std::string kPathTemp{"var/tmp/scdetect"};

// Processor identifier separator
const std::string kProcessorIdSep{"::"};
// Filename for processor debug information
const std::string kFnameDebugInfo{"debug_info.json"};

// Template specific default configuration
const std::string kMagnitudeType{"SCDETECT"};
const std::string kOriginMethod{kMagnitudeType};
const std::string kStationMagnitudeType{kMagnitudeType};

constexpr bool kCacheRawWaveforms{true};
constexpr double kBufferMultiplicator{2};
constexpr double kTemplateWaveformResampleMargin{2};

} // namespace settings
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_SETTINGS_H_
