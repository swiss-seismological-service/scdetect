#include "rms.h"

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../settings.h"
#include "../util/memory.h"
#include "../util/util.h"
#include "../util/waveform_stream_id.h"
#include "factory.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

RMSAmplitude::SignalUnit RMSAmplitude::signalUnitFromString(
    const std::string &signalUnit) {
  using SignalUnit = RMSAmplitude::SignalUnit;
  // SEED names: see SEEDManual, ch.5 / 34, p.47
  static const std::unordered_map<std::string, SignalUnit> lookUpTable{
      {"M", SignalUnit::kMeter},
      {"M/S", SignalUnit::kMeterPerSeconds},
      {"M/S**2", SignalUnit::kMeterPerSecondsSquared}};

  return lookUpTable.at(signalUnit);
}

RMSAmplitude::RMSAmplitude(const SignalTimeInfo &timeInfo)
    : _signalTimeInfo{timeInfo} {}

void RMSAmplitude::reset() {
  AmplitudeProcessor::reset();
  _buffer.clear();
}

void RMSAmplitude::computeTimeWindow() {
  assert((environment().picks.size() == 1 && environment().picks.front()));
  auto referenceTime{environment().picks.front()->time().value()};
  setTimeWindow(Core::TimeWindow{referenceTime + _signalTimeInfo.leading,
                                 referenceTime + _signalTimeInfo.trailing});
}

void RMSAmplitude::setFilter(std::unique_ptr<DoubleFilter> filter,
                             Core::TimeSpan initTime) {
  _streamState.filter = std::move(filter);
  _initTime = initTime;

  reset();
}

void RMSAmplitude::setDeconvolutionConfig(const DeconvolutionConfig &config) {
  _deconvolutionConfig = config;
}

const AmplitudeProcessor::DeconvolutionConfig &
RMSAmplitude::deconvolutionConfig() const {
  return _deconvolutionConfig;
}

void RMSAmplitude::setStreamConfig(
    const processing::StreamConfig &streamConfig) {
  _streamConfig = streamConfig;
}

const processing::StreamConfig &RMSAmplitude::streamConfig() const {
  return _streamConfig;
}

processing::WaveformProcessor::StreamState *RMSAmplitude::streamState(
    const Record *record)  // NOLINT(misc-unused-parameters)
{
  return &_streamState;
}

void RMSAmplitude::process(StreamState &streamState, const Record *record,
                           const DoubleArray &filteredData) {
  // TODO(damb): compute SNR
  setStatus(Status::kInProgress, 1);

  _bufferedTimeWindow = streamState.dataTimeWindow;
  preprocessData(_streamState, _streamConfig, _deconvolutionConfig, _buffer);

  auto amplitude{util::make_smart<Amplitude>()};
  amplitude->value.value = _buffer.rms();

  auto referenceTime{environment().picks.front()->time().value()};
  // time window based amplitude time
  amplitude->time.reference = referenceTime;
  amplitude->time.begin =
      static_cast<double>(referenceTime - _bufferedTimeWindow.startTime());
  amplitude->time.end =
      static_cast<double>(_bufferedTimeWindow.endTime() - referenceTime);

  setStatus(Status::kFinished, 100.0);
  emitAmplitude(record, amplitude);
}

bool RMSAmplitude::fill(processing::StreamState &streamState,
                        const Record *record, DoubleArrayPtr &data) {
  AmplitudeProcessor::fill(streamState, record, data);

  _buffer.append(data->size(), data->typedData());
  return true;
}

void RMSAmplitude::preprocessData(
    StreamState &streamState, const processing::StreamConfig &streamConfig,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {
  // trim buffered data to actual time window
  const auto range{computeIndexRange(timeWindow())};
  assert(range.begin < range.end);
  _buffer.setData(range.end - range.begin, _buffer.typedData() + range.begin);

  _bufferedTimeWindow = timeWindow();

  // remove response and apply gain
  auto sensor{streamConfig.sensor()};
  if (!sensor || !sensor->response()) {
    setStatus(Status::kMissingResponse, 0);
    return;
  }

  if (streamConfig.gain == 0) {
    setStatus(Status::kMissingGain, -1);
    return;
  }

  SignalUnit signalUnit;
  try {
    signalUnit = signalUnitFromString(sensor->unit());
  } catch (std::out_of_range &) {
    setStatus(Status::kIncompatibleUnit, 0);
    return;
  }

  const auto numberOfIntegrations{util::asInteger(signalUnit)};
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

  // XXX(damb): `streamConfig` is not modified
  const_cast<Processing::Stream &>(streamConfig).applyGain(data);
}

AmplitudeProcessor::IndexRange RMSAmplitude::computeIndexRange(
    const Core::TimeWindow &tw) const {
  assert((_streamState.samplingFrequency));
  return IndexRange{
      static_cast<std::size_t>(
          (tw.startTime() - _bufferedTimeWindow.startTime()) *
          _streamState.samplingFrequency),
      static_cast<std::size_t>(
          static_cast<double>(tw.endTime() - _bufferedTimeWindow.startTime()) *
          _streamState.samplingFrequency)};
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
