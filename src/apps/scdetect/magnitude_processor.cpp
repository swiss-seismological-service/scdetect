#include "magnitude_processor.h"

#include <seiscomp/utils/units.h>

#include <memory>

#include "log.h"
#include "magnitude/decorator/range.h"
#include "magnitude/util.h"
#include "util/memory.h"

namespace Seiscomp {
namespace detect {

MagnitudeProcessor::BaseException::BaseException(const std::string& msg,
                                                 Status status)
    : Processor::BaseException{msg}, _status{status} {}

MagnitudeProcessor::BaseException::BaseException()
    : Processor::BaseException{"base magnitude processor exception"} {}

void MagnitudeProcessor::BaseException::setStatus(
    MagnitudeProcessor::Status status) {
  _status = status;
}

MagnitudeProcessor::Status MagnitudeProcessor::BaseException::status() const {
  return _status;
}

const std::string& MagnitudeProcessor::type() const { return _type; }

std::string MagnitudeProcessor::amplitudeType() const { return type(); }

double MagnitudeProcessor::compute(const DataModel::Amplitude* amplitude) {
  return computeMagnitude(amplitude);
}

void MagnitudeProcessor::finalize(
    DataModel::StationMagnitude* magnitude) const {}

double MagnitudeProcessor::convertAmplitude(
    const DataModel::Amplitude* amplitude,
    const std::string& targetAmplitudeUnit) const {
  const auto& amplitudeUnit{amplitude->unit()};
  if (amplitudeUnit.empty()) {
    throw MagnitudeProcessor::BaseException{"missing amplitude unit",
                                            Status::kInvalidAmplitudeUnit};
  }

  if (amplitudeUnit == targetAmplitudeUnit) {
    return amplitude->amplitude().value();
  }

  auto converter{Util::UnitConverter::get(amplitudeUnit)};
  if (!converter) {
    throw MagnitudeProcessor::BaseException{"invalid amplitude unit",
                                            Status::kInvalidAmplitudeUnit};
  }

  // convert to SI
  double amplitudeSI{converter->convert(amplitude->amplitude().value())};

  converter = Util::UnitConverter::get(targetAmplitudeUnit);
  if (!converter) {
    throw MagnitudeProcessor::BaseException{"invalid target amplitude unit",
                                            Status::kInvalidAmplitudeUnit};
  }

  double targetAmplitude{converter->revert(amplitudeSI)};
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Converted amplitude from %f %s to %f %s",
                               amplitude->amplitude().value(),
                               amplitudeUnit.c_str(), targetAmplitude,
                               targetAmplitudeUnit.c_str());
  return targetAmplitude;
}

void MagnitudeProcessor::setType(std::string type) { _type = type; }

/* ------------------------------------------------------------------------- */
bool MagnitudeProcessor::Factory::registerFactory(const std::string& id,
                                                  CallbackType callback) {
  return AdaptedFactory::registerFactory(id, callback);
}

bool MagnitudeProcessor::Factory::unregisterFactory(const std::string& id) {
  return AdaptedFactory::unregisterFactory(id);
}

std::unique_ptr<MagnitudeProcessor> MagnitudeProcessor::Factory::create(
    const DataModel::Amplitude* amplitude, const std::string& id,
    const std::string& processorId) {
  auto ret{AdaptedFactory::create(amplitude->type())};
  if (!ret) {
    ret = AdaptedFactory::create(id);
  }

  // configure template family based magnitude processors
  if (auto ptr{dynamic_cast<magnitude::TemplateFamilyBased*>(ret.get())}) {
    const auto detectorId{magnitude::extractDetectorId(amplitude)};
    if (!detectorId) {
      return nullptr;
    }
    const auto sensorLocationId{magnitude::extractSensorLocationId(amplitude)};
    if (!sensorLocationId) {
      return nullptr;
    }

    std::shared_ptr<TemplateFamily> templateFamily;
    if (!configureTemplateFamily(ptr, amplitude, *detectorId, *sensorLocationId,
                                 ret->type(), templateFamily)) {
      return nullptr;
    }

    SCDETECT_LOG_DEBUG_TAGGED(
        processorId, "Configured processor with template family: id=%s",
        templateFamily->id().c_str());

    if (!configureLimits(ret, *detectorId, *sensorLocationId, templateFamily)) {
      return nullptr;
    }
  }

  if (ret) {
    ret->setId(processorId);
  }

  return ret;
}

void MagnitudeProcessor::Factory::registerTemplateFamily(
    std::unique_ptr<TemplateFamily>&& templateFamily) {
  std::shared_ptr<TemplateFamily> shared{std::move(templateFamily)};
  for (const auto& member : *shared) {
    if (member.referencesDetector()) {
      templateFamilies()[*member.config.detectorId]
                        [member.config.sensorLocationId] = shared;
    }
  }
}

void MagnitudeProcessor::Factory::unregisterTemplateFamily(
    const std::string& detectorId, const std::string& sensorLocationId) {
  templateFamilies()[detectorId].erase(sensorLocationId);
  if (templateFamilies()[detectorId].empty()) {
    templateFamilies().erase(detectorId);
  }
}

bool MagnitudeProcessor::Factory::configureTemplateFamily(
    magnitude::TemplateFamilyBased* processor,
    const DataModel::Amplitude* amplitude, const DetectorId& detectorId,
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

bool MagnitudeProcessor::Factory::configureLimits(
    std::unique_ptr<MagnitudeProcessor>& ret, const DetectorId& detectorId,
    const SensorLocationId& sensorLocationId,
    const std::shared_ptr<TemplateFamily>& templateFamily) {
  ret = util::make_unique<magnitude::decorator::MagnitudeRange>(std::move(ret));
  if (auto ptr{
          dynamic_cast<magnitude::decorator::MagnitudeRange*>(ret.get())}) {
    for (const auto& member : *templateFamily) {
      if (member.referencesDetector() &&
          *member.config.detectorId == detectorId &&
          member.config.sensorLocationId == sensorLocationId) {
        ptr->addLimits(*member.config.detectorId,
                       member.config.sensorLocationId, member.config.lowerLimit,
                       member.config.upperLimit);
        break;
      }
    }
    return true;
  }
  return false;
}

MagnitudeProcessor::Factory::TemplateFamilies&
MagnitudeProcessor::Factory::templateFamilies() {
  static TemplateFamilies* ret{new TemplateFamilies{}};
  return *ret;
}

}  // namespace detect
}  // namespace Seiscomp
