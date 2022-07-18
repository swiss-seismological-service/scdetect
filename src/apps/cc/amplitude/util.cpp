#include "util.h"

#include <seiscomp/datamodel/origin.h>

#include <boost/algorithm/string/join.hpp>
#include <memory>

#include "../amplitude_processor.h"
#include "../exception.h"
#include "../settings.h"
#include "../util/memory.h"

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

}  // namespace util
}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
