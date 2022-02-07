#include "detector_impl.h"

#include <seiscomp/core/strings.h>

#include <algorithm>
#include <cassert>
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
#include "../util/util.h"
#include "arrival.h"

namespace Seiscomp {
namespace detect {
namespace detector {

const Core::TimeSpan DetectorImpl::_linkerSafetyMargin{1.0};

DetectorImpl::DetectorImpl(const DataModel::OriginCPtr &origin)
    : Processor{}, _origin{origin} {
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

DetectorImpl::Status DetectorImpl::status() const { return _status; }

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

void DetectorImpl::setMergingStrategy(
    linker::MergingStrategy::Type mergingStrategyTypeId) {
  _linker.setMergingStrategy(mergingStrategyTypeId);
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
  ProcessorState p{loc, Core::TimeWindow{}, arrival.pick.time, std::move(proc)};
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
  using pair_type = ProcessorStates::value_type;
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
  if (status() == Status::kTerminated) {
    throw BaseException{"error while processing: status=" +
                        std::to_string(util::asInteger(Status::kTerminated))};
  }

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

  _status = Status::kWaitingForData;
}

void DetectorImpl::terminate() {
  _linker.terminate();

  if (triggered()) {
    while (!_resultQueue.empty()) {
      const auto &result{_resultQueue.front()};

      if (result.fit > _currentResult.value().fit &&
          result.processorCount() >= _currentResult.value().processorCount()) {
        _currentResult = result;
      }

      _resultQueue.pop_front();
    }

    Result prepared;
    prepareResult(*_currentResult, prepared);
    emitResult(prepared);

  } else if (!_triggerDuration) {
    while (!_resultQueue.empty()) {
      const auto &result{_resultQueue.front()};
      _currentResult = result;
      _resultQueue.pop_front();

      Result prepared;
      prepareResult(*_currentResult, prepared);
      emitResult(prepared);

      _currentResult = boost::none;
    }
  }

  _status = Status::kTerminated;
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
  const auto &earliestArrivalTemplateResult{sorted.at(0)};
  const auto &pickTime{earliestArrivalTemplateResult.result.arrival.pick.time};

  bool newTrigger{false};
  bool updatedResult{false};
  if (result.fit > triggerOnThreshold) {
    if (!_currentResult) {
      _currentResult = result;

      // enable trigger
      if (_triggerDuration && *_triggerDuration > Core::TimeSpan{0.0}) {
        SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggering) %s",
                                     result.debugString().c_str());

        _triggerProcId = earliestArrivalTemplateResult.processorId;
        _triggerEnd = pickTime + *_triggerDuration;
        // XXX(damb): A side-note on trigger facilities when it comes to the
        // linker:
        // - The linker processes only those template results which are fed to
        // the linker i.e. the linker is not aware of the the fact whether a
        // template waveform processor is enabled or disabled, respectively.
        // - Thus, the detector processor can force the linker into a *triggered
        // state* by only feeding data to those processors which are part of the
        // triggering event.
        disableProcessorsNotContributing(result);

        newTrigger = true;
      } else {
        SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result %s",
                                     result.debugString().c_str());
      }
    } else if (triggered() && (pickTime <= *_triggerEnd) &&
               result.fit > _currentResult.value().fit &&
               result.processorCount() >=
                   _currentResult.value().processorCount()) {
      SCDETECT_LOG_DEBUG_PROCESSOR(this,
                                   "Detector result (triggered, updating) %s",
                                   result.debugString().c_str());
      _currentResult = result;

      updatedResult = true;

      if (_triggerProcId != earliestArrivalTemplateResult.processorId) {
        _triggerProcId = earliestArrivalTemplateResult.processorId;
        _triggerEnd = pickTime + *_triggerDuration;
        disableProcessorsNotContributing(result);
      }
    }
  }

  bool expired{false};
  if (triggered()) {
    expired = pickTime > *_triggerEnd;

    if (!expired && !newTrigger && !updatedResult &&
        result.fit <= _currentResult.value().fit &&
        result.fit >= triggerOffThreshold) {
      SCDETECT_LOG_DEBUG_PROCESSOR(this,
                                   "Detector result (triggered, dropped) %s",
                                   result.debugString().c_str());
    }

    // disable trigger if required
    if (expired || result.fit < triggerOffThreshold) {
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
  if (expired && result.fit > triggerOnThreshold && *_currentResult != result) {
    SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggering) %s",
                                 result.debugString().c_str());

    _currentResult = result;

    _triggerProcId = earliestArrivalTemplateResult.processorId;
    _triggerEnd = pickTime + *_triggerDuration;
  }
}

void DetectorImpl::disableProcessorsNotContributing(
    const linker::Association &result) {
  // disable those processors not contributing to the triggering event
  std::vector<std::string> contributing;
  std::transform(std::begin(result.results), std::end(result.results),
                 std::back_inserter(contributing),
                 [](const linker::Association::TemplateResults::value_type &p) {
                   return p.first;
                 });
  std::for_each(
      std::begin(_processors), std::end(_processors),
      [](ProcessorStates::value_type &p) { p.second.processor->disable(); });
  std::for_each(std::begin(contributing), std::end(contributing),
                [this](const std::string &proc_id) {
                  _processors.at(proc_id).processor->enable();
                });
}

std::string DetectorImpl::triggerProcessorId(
    const linker::Association &result) {
  // determine the processor with the earliest arrival
  return sortByArrivalTime(result).at(0).processorId;
}

void DetectorImpl::prepareResult(const linker::Association &linkerResult,
                                 DetectorImpl::Result &result) const {
  auto sorted{sortByArrivalTime(linkerResult)};

  const auto &referenceResult{sorted.at(0)};
  const auto &referenceArrival{referenceResult.result.arrival};
  const auto referenceMatchResult{referenceResult.result.matchResult};
  const auto &referenceStartTime{referenceMatchResult->timeWindow.startTime()};
  const auto referenceLag{
      static_cast<double>(referenceArrival.pick.time - referenceStartTime)};

  std::unordered_set<std::string> usedChas;
  std::unordered_set<std::string> usedStas;
  std::vector<double> alignedArrivalOffsets;

  DetectorImpl::Result::TemplateResults templateResults;
  for (const auto &result : sorted) {
    const auto &procId{result.processorId};
    const auto &templateResult{result.result};
    const auto &proc{_processors.at(procId)};
    if (templateResult.matchResult) {
      const auto matchResult{templateResult.matchResult};
      const auto &startTime{matchResult->timeWindow.startTime()};

      auto arrivalOffset{templateResult.arrival.pick.time -
                         referenceArrival.pick.time};
      // compute alignment correction using the POT offset (required, since
      // traces to be cross-correlated cannot be guaranteed to be aligned to
      // sub-sampling interval accuracy)
      const auto &alignmentCorrection{referenceStartTime + arrivalOffset -
                                      startTime};

      alignedArrivalOffsets.push_back(
          static_cast<double>(templateResult.arrival.pick.time - startTime) -
          alignmentCorrection - referenceLag);

      templateResults.emplace(
          templateResult.arrival.pick.waveformStreamId,
          DetectorImpl::Result::TemplateResult{
              templateResult.arrival, proc.sensorLocation,
              proc.processor->templateWaveform().startTime(),
              proc.processor->templateWaveform().endTime(),
              proc.templateWaveformReferenceTime, procId});
      usedChas.emplace(templateResult.arrival.pick.waveformStreamId);
      usedStas.emplace(proc.sensorLocation.stationId);
    }
  }

  // compute origin time
  const auto &arrivalOffsetCorrection{
      util::cma(alignedArrivalOffsets.data(), alignedArrivalOffsets.size())};
  const auto &referenceOriginArrivalOffset{referenceArrival.pick.offset};
  result.originTime = referenceStartTime + Core::TimeSpan{referenceLag} -
                      referenceOriginArrivalOffset +
                      Core::TimeSpan{arrivalOffsetCorrection};
  result.fit = linkerResult.fit;
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
  // enable processors
  for (auto &procPair : _processors) {
    procPair.second.processor->enable();
  }

  resetTrigger();
}

void DetectorImpl::resetTrigger() {
  _triggerProcId = boost::none;
  _triggerEnd = boost::none;
}

void DetectorImpl::resetProcessors() {
  std::for_each(std::begin(_processors), std::end(_processors),
                [](ProcessorStates::value_type &p) {
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
    auto msg{Core::stringify(
        "Failed to match template (proc_id=%s). Reason: status=%d, "
        "statusValue=%f",
        p.processor->id().c_str(), util::asInteger(status), statusValue)};

    throw TemplateMatchingError{msg};
  }

  _linker.feed(processor, std::move(result));
}

void DetectorImpl::storeLinkerResult(const linker::Association &linkerResult) {
  _resultQueue.emplace_back(linkerResult);
}

std::vector<DetectorImpl::TemplateResult> DetectorImpl::sortByArrivalTime(
    const linker::Association &linkerResult) {
  std::vector<TemplateResult> ret;
  for (const auto &resultPair : linkerResult.results) {
    ret.push_back({resultPair.second, resultPair.first});
  }
  std::sort(std::begin(ret), std::end(ret),
            [](const TemplateResult &lhs, const TemplateResult &rhs) {
              return lhs.result.arrival.pick.time <
                     rhs.result.arrival.pick.time;
            });
  return ret;
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
