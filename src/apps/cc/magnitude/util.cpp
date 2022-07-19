#include "util.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/constants.hpp>
#include <boost/algorithm/string/split.hpp>

#include "../settings.h"
#include "../util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

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
    if (comment->id() == settings::kAmplitudeStreamsCommentId &&
        !comment->text().empty()) {
      waveformStreamIds = comment->text();
      break;
    }
  }
  if (waveformStreamIds.empty()) {
    return boost::none;
  }

  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, waveformStreamIds,
                          boost::is_any_of(settings::kWaveformStreamIdSep),
                          boost::algorithm::token_compress_on);
  if (tokens.empty()) {
    return boost::none;
  }

  std::string sensorLocationId;
  for (const auto& token : tokens) {
    auto tmp{util::getSensorLocationStreamId(util::WaveformStreamID{token})};
    if (sensorLocationId.empty()) {
      sensorLocationId = tmp;
    } else if (sensorLocationId != tmp) {
      return boost::none;
    }
  }

  return sensorLocationId;
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
