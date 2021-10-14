#include "range.h"

#include "../../settings.h"
#include "util.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {
namespace decorator {

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

}  // namespace decorator
}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
