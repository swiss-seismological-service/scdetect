#include "mrelative.h"

#include <seiscomp/datamodel/waveformstreamid.h>

#include <cassert>
#include <memory>
#include <vector>

#include "../settings.h"
#include "../util/memory.h"
#include "../util/waveform_stream_id.h"
#include "util.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

const MRelative::CombiningStrategy MRelative::median =
    [](const std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> &amplitudes,
       detect::AmplitudeProcessor::Amplitude &combined) {
      assert(!amplitudes.empty());
      std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> sorted(
          static_cast<std::size_t>(
              std::ceil(static_cast<double>(amplitudes.size()) / 2)));
      std::partial_sort_copy(
          std::begin(amplitudes), std::end(amplitudes), std::begin(sorted),
          std::end(sorted),
          [](const detect::AmplitudeProcessor::AmplitudeCPtr &lhs,
             const detect::AmplitudeProcessor::AmplitudeCPtr &rhs) {
            return lhs->value.value < rhs->value.value;
          });

      combined = *sorted.back();
    };

MRelative::MRelative(
    std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying)
    : CombiningAmplitudeProcessor{std::move(underlying), MRelative::median} {
  assert((detect::util::isUniqueSensorLocation(associatedWaveformStreamIds())));
  setType("MRelative");
  setUnit("");
}

void MRelative::finalize(DataModel::Amplitude *amplitude) const {
  CombiningAmplitudeProcessor::finalize(amplitude);

  amplitude->setType(type());
  amplitude->setUnit(unit());

  // waveform stream identifier
  std::vector<std::string> tokens;
  auto waveformStreamIds{associatedWaveformStreamIds()};
  assert(!waveformStreamIds.empty());
  detect::util::tokenizeWaveformStreamId(waveformStreamIds.front(), tokens);
  assert((tokens.size() == 4));
  amplitude->setWaveformID(DataModel::WaveformStreamID{
      tokens[0], tokens[1], tokens[2],
      detect::util::getBandAndSourceCode(tokens[3]), ""});

  // forward reference of the detector which declared the origin
  try {
    auto comment{util::createDetectorIdComment(this)};
    amplitude->add(comment.release());
  } catch (const Exception &) {
  }

  std::vector<std::string> pickPublicIds;
  for (const auto &proc : *this) {
    const auto &env{proc.environment()};
    for (const auto &pick : env.picks) {
      pickPublicIds.emplace_back(pick->publicID());
    }
  }
  // pick public identifiers
  {
    auto comment{detect::util::make_smart<DataModel::Comment>()};
    comment->setId(settings::kAmplitudePicksCommentId);
    comment->setText(
        boost::algorithm::join(pickPublicIds, settings::kPublicIdSep));
    amplitude->add(comment.get());
  }

  // used underlying waveform stream identifiers
  {
    auto comment{util::createAssociatedWaveformStreamIdComment(this)};
    amplitude->add(comment.release());
  }
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
