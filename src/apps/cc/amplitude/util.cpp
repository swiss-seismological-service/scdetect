#include "util.h"

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/comment.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include <boost/algorithm/string/join.hpp>
#include <memory>
#include <string>
#include <vector>

#include "../amplitude_processor.h"
#include "../exception.h"
#include "../settings.h"
#include "../util/memory.h"
#include "../util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {
namespace util {

std::unique_ptr<DataModel::Comment> createAssociatedWaveformStreamIdComment(
    const AmplitudeProcessor* amplitudeProcessor) {
  auto ret{detect::util::make_unique<DataModel::Comment>()};
  ret->setId(settings::kAmplitudeStreamsCommentId);
  ret->setText(
      boost::algorithm::join(amplitudeProcessor->associatedWaveformStreamIds(),
                             settings::kWaveformStreamIdSep));
  return ret;
}

std::unique_ptr<DataModel::Comment> createDetectorIdComment(
    const AmplitudeProcessor* amplitudeProcessor) {
  const auto& origin{amplitudeProcessor->environment().hypocenter};
  assert(origin);
  for (std::size_t i = 0; i < origin->commentCount(); ++i) {
    if (origin->comment(i)->id() == settings::kDetectorIdCommentId) {
      auto ret{detect::util::make_unique<DataModel::Comment>()};
      ret->setId(settings::kDetectorIdCommentId);
      ret->setText(origin->comment(i)->text());
      return ret;
    }
  }
  throw Exception{"no detector associated"};
}

void setWaveformStreamId(const AmplitudeProcessor* amplitudeProcessor,
                         DataModel::Amplitude& amplitude) {
  auto waveformStreamIds{amplitudeProcessor->associatedWaveformStreamIds()};
  assert(!waveformStreamIds.empty());
  std::vector<std::string> tokens;
  detect::util::tokenizeWaveformStreamId(waveformStreamIds.front(), tokens);
  assert((tokens.size() == 4));
  amplitude.setWaveformID(DataModel::WaveformStreamID{
      tokens[0], tokens[1], tokens[2],
      detect::util::getBandAndSourceCode(tokens[3]), ""});
}

}  // namespace util
}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
