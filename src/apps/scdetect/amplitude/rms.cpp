#include "rms.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

RMSAmplitude::RMSAmplitude(const std::string &id)
    : ReducingAmplitudeProcessor{id} {}

void RMSAmplitude::preprocessData(
    StreamState &streamState, Processing::Sensor *sensor,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {
  if (!sensor || !sensor->response()) {
    setStatus(Status::kMissingResponse, 0);
    return;
  }

  SignalUnit signalUnit;
  try {
    signalUnit = signalUnitFromString(sensor->unit());
  } catch (std::out_of_range &) {
    setStatus(Status::kIncompatibleUnit, 0);
    return;
  }

  const auto numberOfIntegrations{utils::asInteger(signalUnit)};
  if (deconvolutionConfig.enabled) {
    if (!deconvolveData(streamState, sensor->response(), deconvolutionConfig,
                        numberOfIntegrations, data)) {
      setStatus(Status::kDeconvolutionFailed, 0);
      return;
    }
  } else {
    if (numberOfIntegrations > 0) {
      // currently, integration is not supported without deconvolution
      setStatus(Status::kError, 0);
      return;
    } else if (numberOfIntegrations < 0) {
      deriveData(streamState, std::abs(numberOfIntegrations), data);
    }
  }
}

DoubleArrayCPtr RMSAmplitude::reduceAmplitudeData(
    const std::vector<DoubleArrayCPtr> &data,
    const std::vector<NoiseInfo> &noiseInfos, const IndexRange &idxRange) {
  if (data.size() != noiseInfos.size()) {
    return nullptr;
  }

  const auto numberOfStreams{data.size()};

  std::vector<double> samples;
  for (size_t i = idxRange.begin; i <= idxRange.end; ++i) {
    double rms{0};
    for (size_t j = 0; j < numberOfStreams; ++j) {
      rms += (data[j]->get(i) - noiseInfos[j].offset) *
             (data[j]->get(i) - noiseInfos[j].offset);
    }
    samples.push_back(sqrt(rms));
  }

  return utils::make_smart<DoubleArray>(static_cast<int>(samples.size()),
                                        samples.data());
}

void RMSAmplitude::computeAmplitude(const DoubleArray &data,
                                    const IndexRange &idxRange,
                                    AmplitudeProcessor::Amplitude &amplitude) {
  auto rangeBegin{data.begin() + idxRange.begin};
  auto rangeEnd{data.begin() + idxRange.end};

  // XXX(damb): currently, the amplitude is computed as the max element-wise
  // RMS
  auto it{std::max_element(rangeBegin, rangeEnd)};
  if (it == data.end()) {
    setStatus(Status::kError, 0);
    return;
  }
  amplitude.amplitude.value = *it;

  // TODO(damb): compute SNR
}

/* ------------------------------------------------------------------------- */
RMSAmplitude::SignalUnit signalUnitFromString(const std::string &signalUnit) {
  using SignalUnit = RMSAmplitude::SignalUnit;
  // SEED names: see SEEDManual, ch.5 / 34, p.47
  static const std::unordered_map<std::string, SignalUnit> lookUpTable{
      {"M", SignalUnit::kMeter},
      {"M/S", SignalUnit::kMeterPerSeconds},
      {"M/S**2", SignalUnit::kMeterPerSecondsSquared}};

  return lookUpTable.at(signalUnit);
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
