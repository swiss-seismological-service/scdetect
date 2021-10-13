#include "regression.h"

#include <seiscomp/core/strings.h>

#include <cfenv>
#include <cmath>
#include <cstddef>
#include <vector>

#include "../log.h"
#include "../settings.h"
#include "../util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

boost::optional<std::string> getSensorLocationStreamIdFromAmplitude(
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

/* ------------------------------------------------------------------------- */
RegressionMagnitude::RegressionMagnitude(const std::string& id)
    : MagnitudeProcessor{id} {}

void RegressionMagnitude::add(
    const std::vector<RegressionMagnitude::AmplitudeMagnitude>&
        amplitudeMagnitudes) {
  _amplitudeMagnitudes.insert(std::end(_amplitudeMagnitudes),
                              std::begin(amplitudeMagnitudes),
                              std::end(amplitudeMagnitudes));
}

void RegressionMagnitude::add(
    const RegressionMagnitude::AmplitudeMagnitude& amplitudeMagnitude) {
  _amplitudeMagnitudes.push_back(amplitudeMagnitude);
}

void RegressionMagnitude::reset() { _amplitudeMagnitudes.clear(); }

/* ------------------------------------------------------------------------- */
MwFixedSlopeRegressionMagnitude::MwFixedSlopeRegressionMagnitude(
    const std::string& id)
    : FixedSlopeRegressionMagnitude{id} {
  _type = "Mw";
}

MLFixedSlopeRegressionMagnitude::MLFixedSlopeRegressionMagnitude(
    const std::string& id)
    : FixedSlopeRegressionMagnitude{id} {
  _type = "ML";
}

/* ------------------------------------------------------------------------- */
MagnitudeRange::MagnitudeOutOfRange::MagnitudeOutOfRange()
    : MagnitudeProcessor::BaseException{"magnitude out of range"} {}

MagnitudeRange::MagnitudeRange(MagnitudeProcessor* processor,
                               const std::string& id)
    : Decorator{processor, id} {}

void MagnitudeRange::add(const std::string& detectorId,
                         const std::string& sensorLocationId,
                         const boost::optional<double>& lower,
                         const boost::optional<double> upper) {
  _ranges[detectorId][sensorLocationId] = Range{lower, upper};
}

double MagnitudeRange::compute(const DataModel::Amplitude* amplitude) {
  auto magnitude{Decorator::compute(amplitude)};

  // extract the amplitude's associated detector
  std::string detectorId;
  for (std::size_t i = 0; i < amplitude->commentCount(); ++i) {
    auto comment{amplitude->comment(i)};
    if (comment->id() == settings::kDetectorIdCommentId &&
        !comment->text().empty()) {
      detectorId = comment->text();
    }
  }

  // no detector associated
  if (detectorId.empty()) {
    return magnitude;
  }

  // no range associated
  auto dit{_ranges.find(detectorId)};
  if (dit == _ranges.end()) {
    return magnitude;
  }

  auto sensorLocationStreamId{
      getSensorLocationStreamIdFromAmplitude(amplitude)};
  // no sensor location associated
  if (!sensorLocationStreamId) {
    return magnitude;
  }

  auto it{dit->second.find(*sensorLocationStreamId)};
  // no range associated
  if (it == dit->second.end()) {
    return magnitude;
  }

  auto& range{it->second};
  if ((range.begin && range.end && magnitude >= *range.begin &&
       magnitude <= *range.end) ||
      (range.begin && magnitude >= *range.begin) ||
      (range.end && magnitude <= *range.end)) {
    return magnitude;
  }

  return handleMagnitudeOutOfRange(amplitude, magnitude);
}

double MagnitudeRange::handleMagnitudeOutOfRange(
    const DataModel::Amplitude* amplitude, double magnitude) {
  throw MagnitudeOutOfRange{};
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
