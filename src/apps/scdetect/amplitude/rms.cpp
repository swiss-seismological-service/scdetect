#include "rms.h"

#include <seiscomp/datamodel/comment.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional/optional.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "../settings.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

RMSAmplitude::RMSAmplitude(const std::string &id)
    : ReducingAmplitudeProcessor{id} {
  _type = "Mrms";
  _unit = "M/S";
}

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

boost::optional<double> RMSAmplitude::reduceNoiseData(
    const std::vector<DoubleArrayCPtr> &data,
    const std::vector<IndexRange> &idxRanges,
    const std::vector<NoiseInfo> &noiseInfos) {
  // TODO(damb): to be implemented
  return boost::none;
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
  amplitude.value.value = *it;

  // TODO(damb): compute SNR
}

void RMSAmplitude::finalize(DataModel::Amplitude *amplitude) const {
  std::vector<std::string> publicIds;
  for (const auto &pick : _environment.picks) {
    publicIds.push_back(pick->publicID());
  }

  // pick public identifiers
  {
    auto comment{utils::make_smart<DataModel::Comment>()};
    comment->setId("scdetectRMSAmplitudePicks");
    comment->setText(boost::algorithm::join(publicIds, settings::kPublicIdSep));
    amplitude->add(comment.get());
  }

  // waveform stream identifiers
  {
    auto comment{utils::make_smart<DataModel::Comment>()};
    comment->setId("scdetectRMSAmplitudeStreams");
    comment->setText(boost::algorithm::join(utils::map_keys(_streams),
                                            settings::kWaveformStreamIdSep));
    amplitude->add(comment.get());
  }
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