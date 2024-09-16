#include "factory.h"

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/magnitude.h>

#include <cassert>
#include <memory>
#include <stdexcept>
#include <vector>

#include "../log.h"
#include "../util/memory.h"
#include "../util/waveform_stream_id.h"
#include "decorator/range.h"
#include "mlx.h"
#include "mrelative.h"
#include "util.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

Factory::BaseException::BaseException() : Exception{"base factory exception"} {}

std::unique_ptr<MagnitudeProcessor> Factory::createMLx(
    const DataModel::Amplitude& amplitude, const binding::Bindings& bindings,
    const std::string& processorId) {
  assert((amplitude.type() == "MLx"));

  const auto detectorId{magnitude::extractDetectorId(&amplitude)};
  assert(detectorId);

  const auto sensorLocationId{magnitude::extractSensorLocationId(&amplitude)};
  assert(sensorLocationId);

  auto mlx{util::make_unique<MLx>()};

  std::shared_ptr<TemplateFamily> templateFamily;
  if (!configureTemplateFamily(mlx.get(), *detectorId, *sensorLocationId,
                               mlx->type(), templateFamily)) {
    throw BaseException{"failed to configure template family"};
  }

  SCDETECT_LOG_DEBUG_TAGGED(processorId,
                            "Configured processor with template family: id=%s",
                            templateFamily->id().c_str());

  std::unique_ptr<MagnitudeProcessor> ret{mlx.release()};
  if (!configureLimits(ret, *detectorId, *sensorLocationId, templateFamily)) {
    throw BaseException{"failed to configure limits"};
  }

  ret->setId(processorId);

  return ret;
}

std::unique_ptr<MagnitudeProcessor> Factory::createMRelative(
    const DataModel::Amplitude& amplitude, const binding::Bindings& bindings,
    const std::string& processorId) {
  assert((amplitude.type() == "MRelative"));

  const auto detectorId{magnitude::extractDetectorId(&amplitude)};
  if (!detectorId) {
    throw BaseException{"failed to extract detector identifier"};
  }
  const auto sensorLocationStreamId{
      magnitude::extractSensorLocationId(&amplitude, true)};
  if (!sensorLocationStreamId) {
    throw BaseException{"failed to extract sensor location stream identifier"};
  }

  std::vector<std::string> tokens;
  detect::util::tokenizeWaveformStreamId(*sensorLocationStreamId, tokens);
  assert((tokens.size() == 4));

  bool useNetworkMagnitude{true};
  try {
    auto sensorLocationConfig{
        bindings.at(tokens[0], tokens[1], tokens[2], tokens[3])};
    useNetworkMagnitude = sensorLocationConfig.magnitudeProcessingConfig
                              .mrelative.useNetworkMagnitude;
  } catch (const std::out_of_range& e) {
    throw BaseException{"failed to load bindings configuration"};
  }

  // configure template magnitude
  MRelative::TemplateMagnitude templateMagnitude{};
  try {
    if (useNetworkMagnitude) {
      auto mag{networkMagnitudes().at(*detectorId)};
      templateMagnitude.value = mag->magnitude().value();
      SCDETECT_LOG_DEBUG_TAGGED(
          processorId,
          "Configured processor with NetworkMagnitude: public_id=\"%s\"",
          mag->publicID().c_str());
    } else {
      auto mag{stationMagnitudes().at(*detectorId).at(*sensorLocationStreamId)};
      templateMagnitude.value = mag->magnitude().value();
      SCDETECT_LOG_DEBUG_TAGGED(
          processorId,
          "Configured processor with StationMagnitude: public_id=\"%s\"",
          mag->publicID().c_str());
    }
  } catch (const std::out_of_range& e) {
    throw BaseException{"failed to configure template magnitude"};
  }

  auto ret{util::make_unique<MRelative>(templateMagnitude)};
  ret->setId(processorId);
  return ret;
}

void Factory::add(std::unique_ptr<TemplateFamily>&& templateFamily) {
  std::shared_ptr<TemplateFamily> shared{std::move(templateFamily)};
  for (const auto& member : *shared) {
    if (member.referencesDetector()) {
      templateFamilies()[*member.config.detectorId]
                        [member.config.sensorLocationId] = shared;
    }
  }
}

void Factory::removeTemplateFamily(const std::string& detectorId,
                                   const std::string& sensorLocationId) {
  templateFamilies()[detectorId].erase(sensorLocationId);
  if (templateFamilies()[detectorId].empty()) {
    templateFamilies().erase(detectorId);
  }
}

void Factory::add(const std::string& detectorId,
                  const std::string& sensorLocationId,
                  DataModel::StationMagnitudeCPtr magnitude) {
  assert(magnitude);
  stationMagnitudes()[detectorId][sensorLocationId] = std::move(magnitude);
}

void Factory::removeStationMagnitude(const std::string& detectorId,
                                     const std::string& sensorLocationId) {
  stationMagnitudes()[detectorId].erase(sensorLocationId);
  if (stationMagnitudes()[detectorId].empty()) {
    stationMagnitudes().erase(detectorId);
  }
}

void Factory::add(const std::string& detectorId,
                  DataModel::MagnitudeCPtr magnitude) {
  assert(magnitude);
  networkMagnitudes()[detectorId] = std::move(magnitude);
}

void Factory::removeNetworkMagnitude(const std::string& detectorId) {
  networkMagnitudes().erase(detectorId);
}

void Factory::reset() {
  resetCallbacks();

  templateFamilies().clear();
  stationMagnitudes().clear();
  networkMagnitudes().clear();
}

bool Factory::configureTemplateFamily(
    magnitude::TemplateFamilyBased* processor, const DetectorId& detectorId,
    const SensorLocationId& sensorLocationId, const std::string& magnitudeType,
    std::shared_ptr<TemplateFamily>& templateFamily) {
  auto dit{templateFamilies().find(detectorId)};
  if (dit == templateFamilies().end()) {
    return false;
  }
  auto sit{dit->second.find(sensorLocationId)};
  if (sit == dit->second.end()) {
    return false;
  }

  // XXX(damb): currently, the implementation does only allow a single
  // template family of a certain magnitude type per detectorId
  // sensorLocationId combination
  if (sit->second->magnitudeType() != magnitudeType) {
    return false;
  }

  templateFamily = sit->second;
  for (const auto& member : *templateFamily) {
    processor->addAmplitudeMagnitude(member.amplitude, member.magnitude);
  }

  return true;
}

bool Factory::configureLimits(
    std::unique_ptr<MagnitudeProcessor>& ret, const DetectorId& detectorId,
    const SensorLocationId& sensorLocationId,
    const std::shared_ptr<TemplateFamily>& templateFamily) {
  auto decorated{
      util::make_unique<magnitude::decorator::MagnitudeRange>(std::move(ret))};
  for (const auto& member : *templateFamily) {
    if (member.referencesDetector() &&
        *member.config.detectorId == detectorId &&
        member.config.sensorLocationId == sensorLocationId) {
      decorated->addLimits(*member.config.detectorId,
                           member.config.sensorLocationId,
                           member.config.lowerLimit, member.config.upperLimit);
      break;
    }
  }
  ret = std::move(decorated);
  return true;
}

Factory::TemplateFamilies& Factory::templateFamilies() {
  static TemplateFamilies* ret{new TemplateFamilies{}};
  return *ret;
}

Factory::StationMagnitudes& Factory::stationMagnitudes() {
  static StationMagnitudes* ret{new StationMagnitudes{}};
  return *ret;
}

Factory::NetworkMagnitudes& Factory::networkMagnitudes() {
  static NetworkMagnitudes* ret{new NetworkMagnitudes{}};
  return *ret;
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
