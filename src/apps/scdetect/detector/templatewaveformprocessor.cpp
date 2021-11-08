#include "templatewaveformprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "../log.h"
#include "../operator/resample.h"
#include "../resamplerstore.h"
#include "../settings.h"
#include "../util/memory.h"
#include "../waveform.h"
#include "../waveformoperator.h"

namespace Seiscomp {
namespace detect {
namespace detector {

namespace detail {

void LocalMaxima::feed(double coefficient, std::size_t lagIdx) {
  if (!std::isfinite(coefficient)) {
    return;
  }

  if (coefficient < prevCoefficient && notDecreasing) {
    values.push_back({prevCoefficient, --lagIdx});
  }

  notDecreasing = coefficient >= prevCoefficient;
  prevCoefficient = coefficient;
}

}  // namespace detail

TemplateWaveformProcessor::TemplateWaveformProcessor(
    const GenericRecordCPtr &waveform, const std::string filterId,
    const Core::Time &templateStartTime, const Core::Time &templateEndTime,
    const Processor *p)
    : _crossCorrelation{waveform, filterId, templateStartTime,
                        templateEndTime} {}

void TemplateWaveformProcessor::setFilter(Filter *filter,
                                          const Core::TimeSpan &initTime) {
  if (_streamState.filter) delete _streamState.filter;

  _streamState.filter = filter;
  _initTime =
      std::max(initTime, Core::TimeSpan{_crossCorrelation.templateLength()});
}

const Core::TimeWindow &TemplateWaveformProcessor::processed() const {
  return _streamState.dataTimeWindow;
}

void TemplateWaveformProcessor::reset() {
  Filter *tmp{_streamState.filter};

  _streamState = StreamState{};
  if (tmp) {
    _streamState.filter = tmp->clone();
    delete tmp;
  }

  _crossCorrelation.reset();

  WaveformProcessor::reset();
}

void TemplateWaveformProcessor::setTargetSamplingFrequency(double f) {
  if (f > 0) {
    _targetSamplingFrequency = f;
  }
}

boost::optional<double> TemplateWaveformProcessor::targetSamplingFrequency()
    const {
  return _targetSamplingFrequency;
}

boost::optional<const Core::Time> TemplateWaveformProcessor::templateStartTime()
    const {
  return _crossCorrelation.templateStartTime();
}

boost::optional<const Core::Time> TemplateWaveformProcessor::templateEndTime()
    const {
  return _crossCorrelation.templateEndTime();
}

Core::TimeSpan TemplateWaveformProcessor::templateDuration() const {
  return Core::TimeSpan{_crossCorrelation.templateLength()};
}

WaveformProcessor::StreamState &TemplateWaveformProcessor::streamState(
    const Record *record) {
  return _streamState;
}

void TemplateWaveformProcessor::process(StreamState &streamState,
                                        const Record *record,
                                        const DoubleArray &filteredData) {
  const auto n{static_cast<size_t>(filteredData.size())};
  setStatus(Status::kInProgress, 1);

  int startIdx{0};
  Core::Time start{record->timeWindow().startTime()};
  // check if processing start lies within the record
  if (!_streamState.initialized) {
    startIdx = std::max(
        0, static_cast<int>(n) - static_cast<int>(_streamState.receivedSamples -
                                                  _streamState.neededSamples));
    const auto t{static_cast<double>(startIdx) / n};
    start =
        record->startTime() + Core::TimeSpan{record->timeWindow().length() * t};
  }

  detail::LocalMaxima maxima;
  for (size_t i{static_cast<size_t>(startIdx)}; i < n; ++i) {
    maxima.feed(filteredData[i], i);
  }

  if (maxima.values.empty()) {
    return;
  }

  const Core::TimeSpan templateLength{_crossCorrelation.templateLength()};
  const Core::TimeWindow tw{start, record->endTime()};
  auto result{util::make_smart<MatchResult>()};
  for (const auto &m : maxima.values) {
    // take cross-correlation filter delay into account i.e. the template
    // processor's result is referring to a time window shifted to the past
    const auto matchIdx{
        static_cast<int>(m.lagIdx - _crossCorrelation.templateSize() + 1)};
    const auto t{static_cast<double>(matchIdx) / n};

    result->localMaxima.push_back(
        MatchResult::Value{m.coefficient, tw.length() * t});
  }

  result->timeWindow = tw;

  emitResult(record, result.get());
}

bool TemplateWaveformProcessor::fill(detect::StreamState &streamState,
                                     const Record *record,
                                     DoubleArrayPtr &data) {
  if (WaveformProcessor::fill(streamState, record, data)) {
    // cross-correlate filtered data
    _crossCorrelation.apply(data->size(), data->typedData());
    return true;
  }
  return false;
}

void TemplateWaveformProcessor::setupStream(StreamState &streamState,
                                            const Record *record) {
  WaveformProcessor::setupStream(streamState, record);
  const auto f{streamState.samplingFrequency};
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Initialize stream: samplingFrequency=%f",
                               f);
  if (_targetSamplingFrequency && _targetSamplingFrequency != f) {
    SCDETECT_LOG_DEBUG_PROCESSOR(this,
                                 "Reinitialize stream: samplingFrequency=%f",
                                 _targetSamplingFrequency);
    auto resamplingOperator{
        util::make_unique<waveform_operator::ResamplingOperator>(
            RecordResamplerStore::Instance().get(record,
                                                 *_targetSamplingFrequency))};
    setOperator(resamplingOperator.release());

    streamState.samplingFrequency = *_targetSamplingFrequency;
    if (streamState.filter) {
      streamState.filter->setSamplingFrequency(*_targetSamplingFrequency);
    }
  }

  _crossCorrelation.setSamplingFrequency(_targetSamplingFrequency.value_or(f));
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
