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

void RatioAmplitude::computeTimeWindow() {
  assert((environment().picks.size() == 1));

  const auto pickTime{environment().picks.front()->time().value()};
  assert((static_cast<bool>(pickTime)));
  assert((_templateWaveform.referenceTime()));
  const auto leadingReferenceTimeOffset{*_templateWaveform.referenceTime() -
                                        _templateWaveform.raw().startTime()};
  const auto trailingReferenceTimeOffset{_templateWaveform.raw().endTime() -
                                         *_templateWaveform.referenceTime()};

  setTimeWindow(Core::TimeWindow{pickTime - leadingReferenceTimeOffset,
                                 pickTime + trailingReferenceTimeOffset});
}

void RatioAmplitude::setTemplateWaveform(TemplateWaveform templateWaveform) {
  assert(_templateWaveform.referenceTime());
  assert(_templateWaveform.configuredStartTime());
  assert(_templateWaveform.configuredEndTime());

  _templateWaveform = std::move(templateWaveform);
  initTemplateWaveform();
}

processing::WaveformProcessor::StreamState *RatioAmplitude::streamState(
    const Record *record)  // NOLINT(misc-unused-parameters)
{
  return &_streamState;
}

void RatioAmplitude::process(StreamState &streamState, const Record *record,
                             const DoubleArray &filteredData) {
  // TODO(damb): compute SNR
  assert((environment().picks.size() == 1));
  setStatus(Status::kInProgress, 1);

  const auto pickTime{environment().picks.front()->time().value()};
  assert((static_cast<bool>(pickTime)));

  _templateWaveform.setSamplingFrequency(streamState.samplingFrequency);

  const auto leadingTemplateWaveformPickOffset{
      *_templateWaveform.referenceTime() - _templateWaveform.startTime()};
  const auto trailingTemplateWaveformPickOffset{
      _templateWaveform.endTime() - *_templateWaveform.referenceTime()};

  const auto signalStartTime{pickTime - leadingTemplateWaveformPickOffset};
  const auto signalEndTime{pickTime + trailingTemplateWaveformPickOffset};

  const auto bufferBeginTime{streamState.dataTimeWindow.startTime()};
  const auto bufferEndTime{streamState.dataTimeWindow.endTime()};

  assert(signalStartTime >= bufferBeginTime);
  assert(signalEndTime <= bufferEndTime);

  // compute signal offsets
  auto signalBeginIdx{static_cast<std::size_t>(
      (signalStartTime - bufferBeginTime) * streamState.samplingFrequency)};
  auto signalEndIdx{static_cast<std::size_t>(
      static_cast<double>(signalEndTime - bufferBeginTime) *
          streamState.samplingFrequency -
      0.5)};

  if (signalBeginIdx >= signalEndIdx) {
    setStatus(Status::kError, 0);
    return;
  }

  const auto &processingConfig{_templateWaveform.processingConfig()};
  // preprocess the data
  if (processingConfig.detrend) {
    waveform::detrend(_buffer);
  }
  if (processingConfig.demean) {
    waveform::demean(_buffer);
  }

  // XXX(damb): resampling not required; the template waveform is resampled to
  // the stream's sampling frequency

  // filter
  try {
    if (!waveform::filter(
            _buffer, boost::variant2::get<0>(processingConfig.filter).get(),
            streamState.samplingFrequency)) {
      throw Exception{"failed to filter template waveform"};
    }
  } catch (const boost::variant2::bad_variant_access &) {
    auto filter{boost::variant2::get<1>(processingConfig.filter)};
    if (filter && !filter.value().empty()) {
      if (!waveform::filter(_buffer, *filter, streamState.samplingFrequency)) {
        throw Exception{"failed to filter template waveform: filter=" +
                        *filter};
      }
    }
  }

  // slice buffered data
  _buffer.setData(signalEndIdx - signalBeginIdx,
                  _buffer.typedData() + signalBeginIdx);

  if (!(_saturationThreshold && checkIfSaturated(_buffer))) {
    return;
  }

  const auto templateWaveformAbsMax{
      DoubleArray::ConstCast(_templateWaveform.waveform().data())->absMax()};
  if (0 == templateWaveformAbsMax) {
    setStatus(Status::kError, 0);
    return;
  }
  const auto dataAbsMax{_buffer.absMax()};
  const auto ratio{dataAbsMax / templateWaveformAbsMax};
  auto amplitude{util::make_smart<Amplitude>()};
  amplitude->value.value = ratio;
  // time window based amplitude time
  amplitude->time.reference = signalStartTime;
  amplitude->time.end = static_cast<double>(signalEndTime - signalStartTime);

  setStatus(Status::kFinished, 100);
  emitAmplitude(record, amplitude);
}

bool RatioAmplitude::fill(processing::StreamState &streamState,
                          const Record *record, DoubleArrayPtr &data) {
  auto &s = dynamic_cast<WaveformProcessor::StreamState &>(streamState);

  const auto n{static_cast<size_t>(data->size())};
  s.receivedSamples += n;

  // XXX(damb): no filter is applied, here. Data is going to be processed in
  // the same way as the underlying template waveform.
  _buffer.append(data->size(), data->typedData());
  return true;
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