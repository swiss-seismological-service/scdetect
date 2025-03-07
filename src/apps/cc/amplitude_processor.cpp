#include "amplitude_processor.h"

#include <seiscomp/core/genericrecord.h>
#include <seiscomp/math/filter/iirdifferentiate.h>
#include <seiscomp/system/environment.h>
#include <seiscomp/utils/files.h>

#include <cmath>

#include "waveform.h"

namespace Seiscomp {
namespace detect {

AmplitudeProcessor::BaseException::BaseException()
    : BaseException{"base amplitude processor exception"} {}

AmplitudeProcessor::DeconvolutionConfig::DeconvolutionConfig(
    const binding::StreamConfig::DeconvolutionConfig &config)
    : enabled{config.enabled},
      responseTaperLength{config.responseTaperLength},
      minimumResponseTaperFrequency{config.minimumResponseTaperFrequency},
      maximumResponseTaperFrequency{config.maximumResponseTaperFrequency} {}

void AmplitudeProcessor::setResultCallback(
    const PublishAmplitudeCallback &callback) {
  _resultCallback = callback;
}

const std::string &AmplitudeProcessor::type() const { return _type; }

const std::string &AmplitudeProcessor::unit() const { return _unit; }

std::vector<std::string> AmplitudeProcessor::associatedWaveformStreamIds()
    const {
  return std::vector<std::string>{};
}

void AmplitudeProcessor::setEnvironment(
    const DataModel::OriginCPtr &hypocenter,
    const DataModel::SensorLocationCPtr &receiver,
    const std::vector<DataModel::PickCPtr> &picks) {
  _environment.hypocenter = hypocenter;
  _environment.receiver = receiver;
  _environment.picks = picks;
}

const AmplitudeProcessor::Environment &AmplitudeProcessor::environment() const {
  return _environment;
}

void AmplitudeProcessor::finalize(DataModel::Amplitude *amplitude) const {}

void AmplitudeProcessor::setType(std::string type) { _type = std::move(type); }

void AmplitudeProcessor::setUnit(std::string unit) { _unit = std::move(unit); }

bool AmplitudeProcessor::computeNoise(const DoubleArray &data,
                                      const IndexRange &idxRange,
                                      NoiseInfo &noiseInfo) {
  // compute offset and rms within the time window
  std::size_t beginIdx{idxRange.begin};
  std::size_t endIdx{idxRange.end};
  if (beginIdx < 0) {
    beginIdx = 0;
  }
  if (endIdx < 0) {
    return false;
  }
  if (endIdx > static_cast<std::size_t>(data.size())) {
    endIdx = static_cast<std::size_t>(data.size());
  }

  // If noise window is zero return an amplitude and offset of zero as well.
  if (endIdx - beginIdx == 0) {
    noiseInfo.offset = 0;
    noiseInfo.amplitude = 0;
    return true;
  }

  DoubleArrayPtr sliced{
      static_cast<DoubleArray *>(data.slice(beginIdx, endIdx))};
  if (!sliced) {
    return false;
  }

  // compute noise offset as the median
  double offset{sliced->median()};
  // compute rms while removing offset
  double amplitude{2 * sliced->rms(offset)};

  noiseInfo.offset = offset;
  noiseInfo.amplitude = amplitude;

  return true;
}

void AmplitudeProcessor::preprocessData(
    StreamState &streamState, const processing::StreamConfig &streamConfig,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {}

bool AmplitudeProcessor::deconvolveData(StreamState &streamState,
                                        Processing::Response *resp,
                                        const DeconvolutionConfig &config,
                                        int numberOfIntegrations,
                                        DoubleArray &data) {
  waveform::detrend(data);
  // XXX(damb): integration is implemented by means of deconvolution i.e. by
  // means of adding an additional zero to the nominator of the rational
  // transfer function
  if (!resp->deconvolveFFT(
          data, streamState.samplingFrequency, config.responseTaperLength,
          config.minimumResponseTaperFrequency,
          config.maximumResponseTaperFrequency,
          numberOfIntegrations < 0 ? 0 : numberOfIntegrations)) {
    return false;
  }

  if (numberOfIntegrations < 0) {
    if (!deriveData(streamState, abs(numberOfIntegrations), data)) {
      return false;
    }
  }

  return true;
}

bool AmplitudeProcessor::deriveData(StreamState &streamState,
                                    int numberOfDerivations,
                                    DoubleArray &data) {
  while (numberOfDerivations > 0) {
    Math::Filtering::IIRDifferentiate<double> diff;
    diff.setSamplingFrequency(streamState.samplingFrequency);
    diff.apply(data.size(), data.typedData());
    --numberOfDerivations;
  }

  return true;
}

void AmplitudeProcessor::emitAmplitude(const Record *record,
                                       const AmplitudeCPtr &amplitude) {
  if (enabled() && _resultCallback) {
    _resultCallback(this, record, amplitude);
  }
}

}  // namespace detect
}  // namespace Seiscomp
