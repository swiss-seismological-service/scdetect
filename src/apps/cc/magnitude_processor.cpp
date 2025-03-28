#include "magnitude_processor.h"

#include <seiscomp/utils/units.h>

#include <cassert>
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
  assert(amplitude);
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

void MagnitudeProcessor::setType(std::string type) { _type = std::move(type); }

}  // namespace detect
}  // namespace Seiscomp
