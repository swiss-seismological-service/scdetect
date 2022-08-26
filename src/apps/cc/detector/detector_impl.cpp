#include "detector_impl.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../config/validators.h"
#include "../log.h"
#include "../util/floating_point_comparison.h"
#include "../util/math.h"
#include "../util/memory.h"
#include "../util/util.h"
#include "arrival.h"
#include "linker.h"
#include "linker/association.h"
#include "template_waveform_processor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

namespace detail {
TemplateWaveformProcessorIterator::TemplateWaveformProcessorIterator(
    ProcessorStatesType::const_iterator it)
    : ProcessorStatesType::const_iterator{it} {}

const TemplateWaveformProcessor &TemplateWaveformProcessorIterator::operator*()
    const {
  return *ProcessorStatesType::const_iterator::operator*().second.processor;
}

}  // namespace detail

const Core::TimeSpan DetectorImpl::_linkerSafetyMargin{1.0};

DetectorImpl::DetectorImpl(const DataModel::OriginCPtr &origin)
    : _origin{origin} {
  _linker.setResultCallback([this](const linker::Association &res) {
    return storeLinkerResult(res);
  });
}

DetectorImpl::BaseException::BaseException()
    : Processor::BaseException{"base detector exception"} {}

DetectorImpl::ProcessingError::ProcessingError()
    : BaseException{"error while processing"} {}

DetectorImpl::TemplateMatchingError::TemplateMatchingError()
    : ProcessingError{"error while matching template"} {}

void DetectorImpl::setGapInterpolation(bool gapInterpolation) {
  for (auto &procPair : _processors) {
    procPair.second.processor->setGapInterpolation(gapInterpolation);
  }
}

void DetectorImpl::setGapThreshold(const Core::TimeSpan &duration) {
  for (auto &procPair : _processors) {
    procPair.second.processor->setGapThreshold(duration);
  }
}

void DetectorImpl::setGapTolerance(const Core::TimeSpan &duration) {
  for (auto &procPair : _processors) {
    procPair.second.processor->setGapTolerance(duration);
  }
}

const Core::TimeWindow &DetectorImpl::processed() const { return _processed; }

bool DetectorImpl::triggered() const { return static_cast<bool>(_triggerEnd); }

void DetectorImpl::enableTrigger(const Core::TimeSpan &duration) {
  _triggerDuration = duration;
}

void DetectorImpl::disableTrigger() { _triggerDuration = boost::none; }

void DetectorImpl::setTriggerThresholds(double triggerOn, double triggerOff) {
  _thresTriggerOn = triggerOn;
  _linker.setThresAssociation(_thresTriggerOn);

  if (_thresTriggerOn && config::validateXCorrThreshold(triggerOff)) {
    _thresTriggerOff = triggerOff;
  }
}

void DetectorImpl::setArrivalOffsetThreshold(
    const boost::optional<Core::TimeSpan> &thres) {
  _linker.setThresArrivalOffset(thres);
}

boost::optional<Core::TimeSpan> DetectorImpl::arrivalOffsetThreshold() const {
  return _linker.thresArrivalOffset();
}

void DetectorImpl::setMinArrivals(const boost::optional<size_t> &n) {
  _linker.setMinArrivals(n);
}

boost::optional<size_t> DetectorImpl::minArrivals() const {
  return _linker.minArrivals();
}

void DetectorImpl::setMergingStrategy(Linker::MergingStrategy mergingStrategy) {
  _linker.setMergingStrategy(std::move(mergingStrategy));
}

void DetectorImpl::setMaxLatency(
    const boost::optional<Core::TimeSpan> &latency) {
  _maxLatency = latency;
}

boost::optional<Core::TimeSpan> DetectorImpl::maxLatency() const {
  return _maxLatency;
}

size_t DetectorImpl::processorCount() const { return _processors.size(); }

const TemplateWaveformProcessor *DetectorImpl::processor(
    const std::string &processorId) const {
  try {
    return _processors.at(processorId).processor.get();
  } catch (std::out_of_range &) {
  }
  return nullptr;
}

void DetectorImpl::add(std::unique_ptr<TemplateWaveformProcessor> proc,
                       const std::string &waveformStreamId,
                       const Arrival &arrival,
                       const DetectorImpl::SensorLocation &loc,
                       const boost::optional<double> &mergingThreshold) {
  proc->setResultCallback(
      [this](const TemplateWaveformProcessor *processor, const Record *record,
             std::unique_ptr<const TemplateWaveformProcessor::MatchResult>
                 result) {
        storeTemplateResult(processor, record, std::move(result));
      });

  // XXX(damb): Replace the arrival with a *pseudo arrival* i.e. an arrival
  // which is associated with the stream to be processed
  Arrival pseudoArrival{arrival};
  pseudoArrival.pick.waveformStreamId = waveformStreamId;

  _linker.add(proc.get(), pseudoArrival, mergingThreshold);
  const auto onHoldDuration{_maxLatency.value_or(0.0) + proc->initTime() +
                            _linkerSafetyMargin};

  if (_linker.onHold() < onHoldDuration) {
    _linker.setOnHold(onHoldDuration);
  }

  const auto procId{proc->id()};
  detail::ProcessorState p{loc, Core::TimeWindow{}, arrival.pick.time,
                           std::move(proc)};
  _processors.emplace(procId, std::move(p));

  _processorIdx.emplace(waveformStreamId, procId);
}

void DetectorImpl::remove(const std::string &waveformStreamId) {
  auto range{_processorIdx.equal_range(waveformStreamId)};
  for (auto &rit = range.first; rit != range.second; ++rit) {
    _linker.remove(rit->first);

    _processorIdx.erase(rit);
    _processors.erase(rit->second);
  }

  // update linker
  using pair_type = detail::ProcessorStatesType::value_type;
  const auto it{
      std::max_element(std::begin(_processors), std::end(_processors),
                       [](const pair_type &lhs, const pair_type &rhs) {
                         return lhs.second.processor->initTime() <
                                rhs.second.processor->initTime();
                       })};
  if (it == std::end(_processors)) {
    return;
  }

  const auto maxOnHoldDuration{it->second.processor->initTime() +
                               _maxLatency.value_or(0.0) + _linkerSafetyMargin};
  if (maxOnHoldDuration > _linker.onHold()) {
    _linker.setOnHold(maxOnHoldDuration);
  }
}

void DetectorImpl::feed(const Record *record) {
  if (!hasAcceptableLatency(record)) {
    logging::TaggedMessage msg{
        record->streamID(),
        "record exceeds acceptable latency. Dropping record (start=" +
            record->startTime().iso() + ", end=" + record->endTime().iso() +
            ")"};
    SCDETECT_LOG_WARNING_PROCESSOR(this, "%s", logging::to_string(msg).c_str());
    // nothing to do
    return;
  }

  // process data by means of underlying template processors
  if (!process(record)) {
    logging::TaggedMessage msg{
        record->streamID(),
        "error while processing data with template processors"};
    throw ProcessingError{logging::to_string(msg)};
  }

  processResultQueue();

  // overall processed endtime
  Core::TimeWindow processed;
  for (const auto &procPair : _processors) {
    const auto procProcessed{procPair.second.processor->processed()};
    if (!procProcessed) {
      processed.setLength(0);
      break;
    }

    if (_processed && _processed.endTime() < procProcessed.endTime()) {
      processed = processed | procProcessed;
    } else {
      processed = procProcessed;
    }
  }
  _processed = processed;

  if (!triggered()) {
    resetProcessing();
  }
}

void DetectorImpl::reset() {
  _linker.reset();
  resetProcessors();
  resetProcessing();
}

void DetectorImpl::flush() {
  _linker.flush();
  processResultQueue();
  // emit pending result
  if (_currentResult) {
    Result prepared;
    prepareResult(*_currentResult, prepared);
    emitResult(prepared);
  }
  resetProcessing();
}

void DetectorImpl::setResultCallback(const PublishResultCallback &callback) {
  _resultCallback = callback;
}

bool DetectorImpl::process(const Record *record) {
  auto range{_processorIdx.equal_range(record->streamID())};
  for (auto rit{range.first}; rit != range.second; ++rit) {
    const auto &procId{rit->second};
    auto &procState{_processors.at(procId)};

    if (!procState.processor->feed(record)) {
      const auto &status{procState.processor->status()};
      const auto &statusValue{procState.processor->statusValue()};
      logging::TaggedMessage msg{
          record->streamID(),
          "failed to feed data (tw.start=" + record->startTime().iso() +
              ", tw.end=" + record->endTime().iso() +
              ") to processor. Reason: status=" +
              std::to_string(util::asInteger(status)) +
              ", status_value=" + std::to_string(statusValue)};
      SCDETECT_LOG_ERROR_TAGGED(procState.processor->id(), "%s",
                                logging::to_string(msg).c_str());

      return false;
    }

    if (!procState.dataTimeWindowFed) {
      procState.dataTimeWindowFed.setStartTime(record->startTime());
    }
    procState.dataTimeWindowFed.setEndTime(record->endTime());
  }

  return true;
}

bool DetectorImpl::hasAcceptableLatency(const Record *record) {
  if (_maxLatency) {
    return record->endTime() > Core::Time::GMT() - *_maxLatency;
  }

  return true;
}

void DetectorImpl::processResultQueue() {
  while (!_resultQueue.empty()) {
    processLinkerResult(_resultQueue.front());
    _resultQueue.pop_front();
  }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void DetectorImpl::processLinkerResult(const linker::Association &result) {
  const auto triggerOnThreshold{_thresTriggerOn.value_or(-1)};
  const auto triggerOffThreshold{_thresTriggerOff.value_or(1)};

  const auto sorted{sortByArrivalTime(result)};
  const auto originTime{computeOriginTime(result, sorted.at(0))};

  bool newTrigger{false};
  bool updatedResult{false};
  if (result.score > triggerOnThreshold) {
    if (!_currentResult) {
      _currentResult = result;

      // enable trigger
      if (_triggerDuration && *_triggerDuration > Core::TimeSpan{0.0}) {
        SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggering) %s",
                                     result.debugString().c_str());

        _triggerEnd = originTime + *_triggerDuration;

        newTrigger = true;
      } else {
        SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result %s",
                                     result.debugString().c_str());
      }
    } else if (triggered() && (originTime <= *_triggerEnd) &&
               result.score > _currentResult.value().score &&
               result.processorCount() >=
                   _currentResult.value().processorCount()) {
      SCDETECT_LOG_DEBUG_PROCESSOR(this,
                                   "Detector result (triggered, updating) %s",
                                   result.debugString().c_str());
      _currentResult = result;
      _triggerEnd = originTime + *_triggerDuration;

      updatedResult = true;
    }
  }

  bool expired{false};
  if (triggered()) {
    expired = originTime > *_triggerEnd;

    if (!expired && !newTrigger && !updatedResult &&
        result.score >= triggerOffThreshold) {
      SCDETECT_LOG_DEBUG_PROCESSOR(this,
                                   "Detector result (triggered, dropped) %s",
                                   result.debugString().c_str());
    }

    // disable trigger if required
    if (expired || result.score < triggerOffThreshold) {
      resetTrigger();
    }
  }

  // emit detection
  if (!triggered()) {
    Result prepared;
    prepareResult(*_currentResult, prepared);
    emitResult(prepared);
  }

  // re-trigger
  if (expired && result.score > triggerOnThreshold &&
      *_currentResult != result) {
    SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggering) %s",
                                 result.debugString().c_str());

    _currentResult = result;

    _triggerEnd = originTime + *_triggerDuration;
  }

  if (!triggered()) {
    _currentResult = boost::none;
  }
}

void DetectorImpl::prepareResult(const linker::Association &linkerResult,
                                 DetectorImpl::Result &result) const {
  std::unordered_set<std::string> usedChas;
  std::unordered_set<std::string> usedStas;
  DetectorImpl::Result::TemplateResults templateResults;
  for (const auto &templateResultPair : linkerResult.results) {
    const auto &procId{templateResultPair.first};
    const auto &proc{_processors.at(procId)};
    const auto &templateResult{templateResultPair.second};
    assert(templateResult.matchResult);

    const auto matchResult{templateResult.matchResult};

    templateResults.emplace(templateResult.arrival.pick.waveformStreamId,
                            DetectorImpl::Result::TemplateResult{
                                templateResult.arrival, proc.sensorLocation,
                                proc.processor->templateWaveform().startTime(),
                                proc.processor->templateWaveform().endTime(),
                                proc.templateWaveformReferenceTime, procId});
    usedChas.emplace(templateResult.arrival.pick.waveformStreamId);
    usedStas.emplace(proc.sensorLocation.stationId);
  }

  auto sorted{sortByArrivalTime(linkerResult)};
  const auto &referenceResult{sorted.at(0)};
  result.originTime = computeOriginTime(linkerResult, referenceResult);

  result.score = linkerResult.score;
  // template results i.e. theoretical arrivals including some meta data
  result.templateResults = templateResults;
  // number of channels used
  result.numChannelsUsed = usedChas.size();
  // number of stations used
  result.numStationsUsed = usedStas.size();
  // number of channels/stations associated
  std::unordered_set<std::string> associatedStations;
  for (const auto &procPair : _processors) {
    associatedStations.emplace(procPair.second.sensorLocation.stationId);
  }
  result.numChannelsAssociated = _linker.channelCount();
  result.numStationsAssociated = associatedStations.size();
}

void DetectorImpl::emitResult(const DetectorImpl::Result &result) {
  if (_resultCallback) {
    _resultCallback.value()(result);
  }
}

void DetectorImpl::resetProcessing() {
  _currentResult = boost::none;
  resetTrigger();
}

void DetectorImpl::resetTrigger() { _triggerEnd = boost::none; }

void DetectorImpl::resetProcessors() {
  std::for_each(std::begin(_processors), std::end(_processors),
                [](detail::ProcessorStatesType::value_type &p) {
                  p.second.processor->reset();
                  p.second.dataTimeWindowFed = Core::TimeWindow{};
                });
}

void DetectorImpl::storeTemplateResult(
    const TemplateWaveformProcessor *processor, const Record *record,
    std::unique_ptr<const TemplateWaveformProcessor::MatchResult> result) {
  assert((processor && record && result));

  auto &p{_processors.at(processor->id())};
  if (p.processor->finished()) {
    const auto &status{p.processor->status()};
    const auto &statusValue{p.processor->statusValue()};
    auto msg{"failed to match template (proc_id=" + p.processor->id() +
             "). Reason: status=" + std::to_string(util::asInteger(status)) +
             ", statusValue=" + std::to_string(statusValue)};
    throw TemplateMatchingError{msg};
  }

  if (triggered()) {
    bool contributing{_currentResult.value().results.count(processor->id()) ==
                      1};
    if (!contributing) {
      const auto originArrivalOffset{
          _linker.originArrivalOffset(processor->id())};

      const auto matchResultArrivalEndTime{result->timeWindow.endTime() +
                                           originArrivalOffset};
      if (_triggerEnd.value_or(matchResultArrivalEndTime) >
          matchResultArrivalEndTime) {
        // XXX(damb): drop match result
        return;
      }

      const auto matchResultArrivalStartTime{result->timeWindow.startTime() +
                                             originArrivalOffset};
      if (_triggerEnd.value_or(matchResultArrivalStartTime) >
          matchResultArrivalStartTime) {
        // XXX(damb): partly use the match result
        auto overlapping{*_triggerEnd - matchResultArrivalStartTime};
        auto samplingFrequency{
            processor->templateWaveform().samplingFrequency()};
        auto sampleOffset{
            std::floor(static_cast<double>(overlapping) * samplingFrequency)};
        auto slicedMatchResult{
            util::make_unique<TemplateWaveformProcessor::MatchResult>(*result)};
        slicedMatchResult->localMaxima.erase(
            slicedMatchResult->localMaxima.begin(),
            slicedMatchResult->localMaxima.begin() +
                static_cast<int>(sampleOffset) + 1);
      }
    }
  }

  _linker.feed(processor, std::move(result));
}

void DetectorImpl::storeLinkerResult(const linker::Association &linkerResult) {
  _resultQueue.emplace_back(linkerResult);
}

std::vector<linker::Association::TemplateResult>
DetectorImpl::sortByArrivalTime(const linker::Association &linkerResult) {
  std::vector<linker::Association::TemplateResult> ret;
  for (const auto &resultPair : linkerResult.results) {
    ret.push_back({resultPair.second});
  }
  std::sort(std::begin(ret), std::end(ret),
            [](const linker::Association::TemplateResult &lhs,
               const linker::Association::TemplateResult &rhs) {
              return lhs.arrival.pick.time < rhs.arrival.pick.time;
            });
  return ret;
}

Core::Time DetectorImpl::computeOriginTime(
    const linker::Association &linkerResult,
    const linker::Association::TemplateResult &referenceResult) {
  const auto &referenceArrival{referenceResult.arrival};
  const auto referenceMatchResult{referenceResult.matchResult};
  const auto &referenceStartTime{referenceMatchResult->timeWindow.startTime()};
  const auto referenceLag{
      static_cast<double>(referenceArrival.pick.time - referenceStartTime)};

  std::vector<double> alignedArrivalOffsets;
  for (const auto &templateResultPair : linkerResult.results) {
    const auto &templateResult{templateResultPair.second};
    assert(templateResult.matchResult);
    const auto matchResult{templateResult.matchResult};
    const auto &startTime{matchResult->timeWindow.startTime()};

    auto arrivalOffset{templateResult.arrival.pick.time -
                       referenceArrival.pick.time};
    // compute alignment correction using the POT offset (required, since
    // traces to be cross-correlated cannot be guaranteed to be aligned to
    // sub-sampling interval accuracy)
    const auto alignmentCorrection{referenceStartTime + arrivalOffset -
                                   startTime};

    alignedArrivalOffsets.push_back(
        static_cast<double>(templateResult.arrival.pick.time - startTime) -
        alignmentCorrection - referenceLag);
  }

  const auto &arrivalOffsetCorrection{
      util::cma(alignedArrivalOffsets.data(), alignedArrivalOffsets.size())};
  const auto &referenceOriginArrivalOffset{referenceArrival.pick.offset};

  const auto originTime{referenceStartTime + Core::TimeSpan{referenceLag} -
                        referenceOriginArrivalOffset +
                        Core::TimeSpan{arrivalOffsetCorrection}};
  return originTime;
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
