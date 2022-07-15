#include "mlx.h"

#include <seiscomp/datamodel/comment.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>

#include "../settings.h"
#include "../util/memory.h"
#include "../util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

const MLx::CombiningStrategy MLx::max =
    [](const std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> &amplitudes,
       detect::AmplitudeProcessor::Amplitude &combined) {
      assert(!amplitudes.empty());
      auto result{std::max_element(
          std::begin(amplitudes), std::end(amplitudes),
          [](const AmplitudeCPtr &lhs, const AmplitudeCPtr &rhs) {
            return lhs->value.value < rhs->value.value;
          })};

      combined = *(result->get());
    };

MLx::MLx(
    std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying)
    : CombiningAmplitudeProcessor{std::move(underlying), MLx::max} {
  assert((validateUniqueSensorLocation(associatedWaveformStreamIds())));
  setType("MLx");
  setUnit("M/S");
}

void MLx::finalize(DataModel::Amplitude *amplitude) const {
  CombiningAmplitudeProcessor::finalize(amplitude);

  amplitude->setType(type());
  amplitude->setUnit(unit());

  // TODO(damb): modularize code. This code was copied from `MRelative`. To be
  // reused, instead.

  // waveform stream identifier
  std::vector<std::string> tokens;
  auto waveformStreamIds{associatedWaveformStreamIds()};
  assert(!waveformStreamIds.empty());
  util::tokenizeWaveformStreamId(waveformStreamIds.front(), tokens);
  assert((tokens.size() == 4));
  amplitude->setWaveformID(
      DataModel::WaveformStreamID{tokens[0], tokens[1], tokens[2],
                                  util::getBandAndSourceCode(tokens[3]), ""});

  const auto &env{environment()};
  // forward reference of the detector which declared the origin
  {
    const auto &origin{env.hypocenter};
    assert(origin);
    for (std::size_t i = 0; i < origin->commentCount(); ++i) {
      if (origin->comment(i)->id() == settings::kDetectorIdCommentId) {
        auto comment{util::make_smart<DataModel::Comment>()};
        comment->setId(settings::kDetectorIdCommentId);
        comment->setText(origin->comment(i)->text());
        amplitude->add(comment.get());
        break;
      }
    }
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
    auto comment{util::make_smart<DataModel::Comment>()};
    comment->setId(settings::kAmplitudePicksCommentId);
    comment->setText(
        boost::algorithm::join(pickPublicIds, settings::kPublicIdSep));
    amplitude->add(comment.get());
  }

  // used underlying waveform stream identifiers
  {
    auto comment{util::make_smart<DataModel::Comment>()};
    comment->setId(settings::kAmplitudeStreamsCommentId);
    comment->setText(boost::algorithm::join(associatedWaveformStreamIds(),
                                            settings::kWaveformStreamIdSep));
    amplitude->add(comment.get());
  }
}

bool MLx::validateUniqueSensorLocation(
    const std::vector<std::string> &waveformStreamIds) {
  std::unordered_set<std::string> sensorLocationStreamIds;
  for (const auto &waveformStreamId : waveformStreamIds) {
    try {
      sensorLocationStreamIds.emplace(util::getSensorLocationStreamId(
          util::WaveformStreamID(waveformStreamId)));
    } catch (const ValueException &) {
      return false;
    }
  }
  return sensorLocationStreamIds.size() == 1;
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
