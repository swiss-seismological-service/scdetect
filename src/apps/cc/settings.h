#ifndef SCDETECT_APPS_CC_SETTINGS_H_
#define SCDETECT_APPS_CC_SETTINGS_H_

#include <string>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace settings {

// Relative path from the SeisComP installation directory
const std::string kPathFilesystemCache{"var/cache/scdetect/cc"};
// Relative path from the SeisComP installation directory
const std::string kPathTemp{"var/tmp/scdetect/cc"};

// Processor identifier separator
const std::string kProcessorIdSep{"::"};
// Filename for processor debug information
const std::string kFnameDebugInfo{"debug_info.json"};

// Separator for waveform stream identifiers (if multiple waveform stream
// identifiers need to be concatenated)
const std::string kWaveformStreamIdSep{"||"};
// Separator for public identifiers (i.e. publicIDs)
const std::string kPublicIdSep{"||"};
// Separator for configuration lists
const std::string kConfigListSep{","};

const std::string kTemplateWaveformTimeInfoPickCommentId{
    "scdetectTemplateWaveformTimeInfo"};
const std::string kTemplateWaveformTimeInfoPickCommentIdSep{"|"};

const std::string kDetectorIdCommentId{"scdetectDetectorId"};
const std::string kAmplitudeStreamsCommentId{"scdetectAmplitudeStreams"};
const std::string kAmplitudePicksCommentId{"scdetectAmplitudePicks"};

const std::vector<std::string> kValidPrioritizedStationMagnitudeTypes{"MLhc",
                                                                      "MLh"};

constexpr bool kCacheRawWaveforms{true};
constexpr double kTemplateWaveformResampleMargin{2};

constexpr int kObjectThroughputAverageTimeSpan{10};

}  // namespace settings
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_SETTINGS_H_
