#include "ratio.h"

#include <boost/variant2/variant.hpp>
#include <cassert>
#include <cstddef>

#include "../util/memory.h"
#include "../util/util.h"
#include "../util/waveform_stream_id.h"
#include "../waveform.h"
#include "factory.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

RatioAmplitude::RatioAmplitude(TemplateWaveform templateWaveform)
    : _templateWaveform{std::move(templateWaveform)} {
  assert(_templateWaveform.referenceTime());
  assert(_templateWaveform.configuredStartTime());
  assert(_templateWaveform.configuredEndTime());

  initTemplateWaveform();
}

void RatioAmplitude::reset() {
  processing::TimeWindowProcessor::reset();
  _templateWaveform.reset();
  _buffer.clear();
}

void RatioAmplitude::computeTimeWindow() {
  setTimeWindow(computeTimeWindow(_templateWaveform.raw().startTime(),
                                  _templateWaveform.raw().endTime()));
}

void RatioAmplitude::setFilter(std::unique_ptr<DoubleFilter> filter,
                               Core::TimeSpan initTime) {
  auto processingConfig{_templateWaveform.processingConfig()};
  processingConfig.filter = std::move(filter);
  processingConfig.initTime = initTime;
  _templateWaveform.setProcessingConfig(processingConfig);

  _initTime = initTime;

  reset();
}

void RatioAmplitude::setTemplateWaveform(
    const TemplateWaveform &templateWaveform) {
  assert(templateWaveform.referenceTime());
  assert(templateWaveform.configuredStartTime());
  assert(templateWaveform.configuredEndTime());

  _templateWaveform = templateWaveform;
  initTemplateWaveform();
}

processing::WaveformProcessor::StreamState *RatioAmplitude::streamState(
    const Record *record)  // NOLINT(misc-unused-parameters)
{
  return &_streamState;
}

void RatioAmplitude::process(
    StreamState &streamState, const Record *record,
    const DoubleArray &filteredData  // NOLINT(misc-unused-parameters)
) {
  // TODO(damb): compute SNR
  setStatus(Status::kInProgress, 1);

  _bufferedTimeWindow = streamState.dataTimeWindow;

  preprocess();

  if (_saturationThreshold && checkIfSaturated(_buffer)) {
    return;
  }

  const auto templateWaveformAbsMax{std::abs(
      DoubleArray::ConstCast(_templateWaveform.waveform().data())->absMax())};
  if (templateWaveformAbsMax == 0) {
    setStatus(Status::kError, 0);
    return;
  }

  auto amplitude{util::make_smart<Amplitude>()};

  const auto dataAbsMax{std::abs(_buffer.absMax())};
  const auto ratio{dataAbsMax / templateWaveformAbsMax};
  amplitude->value.value = ratio;

  // time window based amplitude time
  amplitude->time.reference = _bufferedTimeWindow.startTime();
  amplitude->time.end = static_cast<double>(_bufferedTimeWindow.endTime() -
                                            _bufferedTimeWindow.startTime());

  setStatus(Status::kFinished, 100.0);
  emitAmplitude(record, amplitude);
}

bool RatioAmplitude::fill(
    processing::StreamState &streamState,
    const Record *record,  // NOLINT(misc-unused-parameters)
    DoubleArrayPtr &data) {
  auto &s = dynamic_cast<WaveformProcessor::StreamState &>(streamState);

  const auto n{static_cast<size_t>(data->size())};
  s.receivedSamples += n;

  // XXX(damb): no filter is applied, here. Data is going to be processed in
  // the same way as the underlying template waveform.
  _buffer.append(data->size(), data->typedData());
  return true;
}

void RatioAmplitude::preprocess() {
  // XXX(damb): make sure the signal time window and the template waveform
  // time window is fully aligned. This includes the raw template waveform,
  // too.
  // trim buffered data to raw time window
  const auto rawTimeWindow{computeTimeWindow(
      _templateWaveform.raw().startTime(), _templateWaveform.raw().endTime())};
  const auto rawIdxRange{computeIndexRange(rawTimeWindow)};

  assert(rawIdxRange.begin < rawIdxRange.end);
  _buffer.setData(rawIdxRange.end - rawIdxRange.begin,
                  _buffer.typedData() + rawIdxRange.begin);

  _bufferedTimeWindow = rawTimeWindow;

  const auto &processingConfig{_templateWaveform.processingConfig()};
  if (processingConfig.detrend) {
    waveform::detrend(_buffer);
  }
  if (processingConfig.demean) {
    waveform::demean(_buffer);
  }

  // XXX(damb): resampling not required; the template waveform is resampled to
  // the stream's sampling frequency, implicitly

  assert((_streamState.samplingFrequency));
  // filter
  try {
    if (!waveform::filter(
            _buffer, boost::variant2::get<0>(processingConfig.filter).get(),
            _streamState.samplingFrequency)) {
      throw BaseException{"failed to filter buffered waveform data"};
    }
  } catch (const boost::variant2::bad_variant_access &) {
    auto filter{boost::variant2::get<1>(processingConfig.filter)};
    if (filter && !filter.value().empty()) {
      if (!waveform::filter(_buffer, *filter, _streamState.samplingFrequency)) {
        throw BaseException{"failed to filter buffered waveform data: filter=" +
                            *filter};
      }
    }
  }

  // trim buffered data to signal time window
  const auto signalTimeWindow{computeTimeWindow(_templateWaveform.startTime(),
                                                _templateWaveform.endTime())};
  const auto signalIdxRange{computeIndexRange(signalTimeWindow)};

  assert(signalIdxRange.begin < signalIdxRange.end);
  _buffer.setData(signalIdxRange.end - signalIdxRange.begin,
                  _buffer.typedData() + signalIdxRange.begin);

  _bufferedTimeWindow = signalTimeWindow;
}

Core::TimeWindow RatioAmplitude::computeTimeWindow(
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

RatioAmplitude::IndexRange RatioAmplitude::computeIndexRange(
    const Core::TimeWindow &tw) const {
  assert((_streamState.samplingFrequency));
  return IndexRange{
      static_cast<std::size_t>(
          (tw.startTime() - _bufferedTimeWindow.startTime()).length() *
          _streamState.samplingFrequency),
      static_cast<std::size_t>(
          (tw.endTime() - _bufferedTimeWindow.startTime()).length() *
          _streamState.samplingFrequency)};
}

void RatioAmplitude::initTemplateWaveform() {
  auto processingConfig{_templateWaveform.processingConfig()};
  processingConfig.detrend = true;
  processingConfig.demean = true;

  _templateWaveform.setProcessingConfig(processingConfig);
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
