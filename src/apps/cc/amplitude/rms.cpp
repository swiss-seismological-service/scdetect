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

RMSAmplitude::RMSAmplitude(const TemplateWaveform &templateWaveform)
    : _templateWaveform{templateWaveform} {
  assert(_templateWaveform.referenceTime());
  assert(_templateWaveform.configuredStartTime());
  assert(_templateWaveform.configuredEndTime());
}

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

void RMSAmplitude::reset() {
  AmplitudeProcessor::reset();
  _templateWaveform.reset();
  _buffer.clear();
}

void RMSAmplitude::computeTimeWindow() {
  setTimeWindow(computeTimeWindow(_templateWaveform.configuredStartTime(),
                                  _templateWaveform.configuredEndTime()));
}

void RMSAmplitude::setFilter(std::unique_ptr<DoubleFilter> filter,
                             Core::TimeSpan initTime) {
  _streamState.filter = std::move(filter);
  _initTime = initTime;

  reset();
}

void RMSAmplitude::setTemplateWaveform(
    const TemplateWaveform &templateWaveform) {
  assert(templateWaveform.referenceTime());
  assert(templateWaveform.configuredStartTime());
  assert(templateWaveform.configuredEndTime());

  _templateWaveform = templateWaveform;
}

const TemplateWaveform &RMSAmplitude::templateWaveform() const {
  return _templateWaveform;
}

void RMSAmplitude::setDeconvolutionConfig(const DeconvolutionConfig &config) {
  _deconvolutionConfig = config;
}

const AmplitudeProcessor::DeconvolutionConfig &
RMSAmplitude::deconvolutionConfig() const {
  return _deconvolutionConfig;
}

void RMSAmplitude::setStreamConfig(const StreamConfig &streamConfig) {
  _streamConfig = streamConfig;
}

const AmplitudeProcessor::StreamConfig &RMSAmplitude::streamConfig() const {
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

  // time window based amplitude time
  amplitude->time.reference = _bufferedTimeWindow.startTime();
  amplitude->time.end = static_cast<double>(_bufferedTimeWindow.endTime() -
                                            _bufferedTimeWindow.startTime());

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
    StreamState &streamState, const StreamConfig &streamConfig,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {
  // trim buffered data to configured template waveform time window
  const auto configuredTemplateWaveformTimeWindow{
      computeTimeWindow(_templateWaveform.configuredStartTime(),
                        _templateWaveform.configuredEndTime())};
  const auto range{computeIndexRange(configuredTemplateWaveformTimeWindow)};

  assert(range.begin < range.end);
  _buffer.setData(range.end - range.begin, _buffer.typedData() + range.begin);

  _bufferedTimeWindow = configuredTemplateWaveformTimeWindow;

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

Core::TimeWindow RMSAmplitude::computeTimeWindow(
    const Core::Time &startTime, const Core::Time &endTime) const {
  assert((environment().picks.size() == 1));

  const auto pickTime{environment().picks.front()->time().value()};
  assert((static_cast<bool>(pickTime)));
  assert((_templateWaveform.referenceTime()));

  const auto leadingPickOffset{*_templateWaveform.referenceTime() - startTime};
  const auto trailingPickOffset{endTime - *_templateWaveform.referenceTime()};

  return Core::TimeWindow{pickTime - leadingPickOffset,
                          pickTime + trailingPickOffset};
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
