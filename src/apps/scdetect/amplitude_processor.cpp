#include "amplitude_processor.h"

#include <seiscomp/core/genericrecord.h>
#include <seiscomp/math/filter/iirdifferentiate.h>
#include <seiscomp/system/environment.h>
#include <seiscomp/utils/files.h>

#include "waveform.h"

namespace Seiscomp {
namespace detect {

void AmplitudeProcessor::setResultCallback(
    const PublishAmplitudeCallback &callback) {
  _resultCallback = callback;
}

void AmplitudeProcessor::setSignalBegin(
    const boost::optional<Core::TimeSpan> &signalBegin) {
  if (!signalBegin) {
    _config.signalBegin = signalBegin;
    return;
  }

  if (*signalBegin > Core::TimeSpan{0.0} &&
      safetyTimeWindow().endTime() - _config.signalEnd.value_or(0.0) >
          safetyTimeWindow().startTime() + *signalBegin) {
    _config.signalBegin = signalBegin;
  }
}

Core::Time AmplitudeProcessor::signalBegin() const {
  return safetyTimeWindow().startTime() + _config.signalBegin.value_or(0.0);
}

void AmplitudeProcessor::setSignalEnd(
    const boost::optional<Core::TimeSpan> &signalEnd) {
  if (!signalEnd) {
    _config.signalEnd = signalEnd;
    return;
  }

  if (*signalEnd > Core::TimeSpan{0.0} &&
      safetyTimeWindow().endTime() - *signalEnd >
          safetyTimeWindow().startTime() + _config.signalBegin.value_or(0.0)) {
    _config.signalEnd = signalEnd;
  }
}

Core::Time AmplitudeProcessor::signalEnd() const {
  return safetyTimeWindow().endTime() - _config.signalEnd.value_or(0.0);
}

const std::string &AmplitudeProcessor::type() const { return _type; }

const std::string &AmplitudeProcessor::unit() const { return _unit; }

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

void AmplitudeProcessor::preprocessData(
    StreamState &streamState, const Processing::Stream &streamConfig,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {}

bool AmplitudeProcessor::computeNoise(const DoubleArray &data,
                                      const IndexRange &idxRange,
                                      NoiseInfo &noiseInfo) {
  // compute offset and rms within the time window
  size_t beginIdx{idxRange.begin}, endIdx{idxRange.end};
  if (beginIdx < 0) beginIdx = 0;
  if (endIdx < 0) return false;
  if (endIdx > static_cast<size_t>(data.size()))
    endIdx = static_cast<size_t>(data.size());

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

  if (offset) noiseInfo.offset = offset;
  if (amplitude) noiseInfo.amplitude = amplitude;

  return true;
}

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
