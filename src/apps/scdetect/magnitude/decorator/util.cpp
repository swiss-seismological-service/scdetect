#include "util.h"

#include <seiscomp/core/strings.h>

#include "../../settings.h"
#include "../../util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {
namespace decorator {

boost::optional<std::string> extractDetectorId(
    const DataModel::Amplitude* amplitude) {
  for (std::size_t i = 0; i < amplitude->commentCount(); ++i) {
    auto comment{amplitude->comment(i)};
    if (comment->id() == settings::kDetectorIdCommentId &&
        !comment->text().empty()) {
      return comment->text();
    }
  }

  return boost::none;
}

boost::optional<std::string> extractSensorLocationId(
    const DataModel::Amplitude* amplitude) {
  std::string waveformStreamIds;
  for (std::size_t i = 0; i < amplitude->commentCount(); ++i) {
    auto comment{amplitude->comment(i)};
    if (comment->id() == settings::kRMSAmplitudeStreamsCommentId &&
        !comment->text().empty()) {
      waveformStreamIds = comment->text();
      break;
    }
  }
  if (waveformStreamIds.empty()) {
    return boost::none;
  }

  std::vector<std::string> tokens;
  Core::split(tokens, waveformStreamIds, settings::kWaveformStreamIdSep.c_str(),
              false);
  if (tokens.empty()) {
    return boost::none;
  }

  std::string sensorLocationId;
  for (const auto& token : tokens) {
    auto tmp{util::WaveformStreamID{token}.sensorLocationStreamId()};
    if (sensorLocationId.empty()) {
      sensorLocationId = tmp;
    } else if (sensorLocationId != tmp) {
      return boost::none;
    }
  }

  return sensorLocationId;
}

}  // namespace decorator
}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
