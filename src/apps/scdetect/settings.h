#ifndef SCDETECT_APPS_SCDETECT_SETTINGS_H_
#define SCDETECT_APPS_SCDETECT_SETTINGS_H_

#include <string>

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

// Separator for waveform stream identifiers (if multiple waveform stream
// identifiers need to be concatenated)
const std::string kWaveformStreamIdSep{"||"};
// Separator for public identifiers (i.e. publicIDs)
const std::string kPublicIdSep{"||"};

// Template specific default configuration
const std::string kMagnitudeType{"DETECT"};
const std::string kStationMagnitudeType{kMagnitudeType};

constexpr bool kCacheRawWaveforms{true};
constexpr double kBufferMultiplicator{2};
constexpr double kTemplateWaveformResampleMargin{2};

}  // namespace settings
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_SETTINGS_H_
