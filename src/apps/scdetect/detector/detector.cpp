#include "detector.h"

#include <seiscomp/core/strings.h>

#include <algorithm>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../config/validators.h"
#include "../log.h"
#include "../util/floating_point_comparison.h"
#include "../util/math.h"
#include "../util/util.h"

namespace Seiscomp {
namespace detect {
namespace detector {

const Core::TimeSpan Detector::_linkerSafetyMargin{1.0};

Detector::Detector(const DataModel::OriginCPtr &origin)
    : Processor{}, _origin{origin} {
  _linker.setResultCallback([this](const linker::Association &res) {
    return storeLinkerResult(res);
  });
}

Detector::~Detector() {}

Detector::BaseException::BaseException()
    : Processor::BaseException{"base detector exception"} {}

Detector::ProcessingError::ProcessingError()
    : BaseException{"error while processing"} {}

Detector::TemplateMatchingError::TemplateMatchingError()
    : ProcessingError{"error while matching template"} {}

Detector::Status Detector::status() const { return _status; }

const Core::TimeWindow &Detector::processed() const { return _processed; }

bool Detector::triggered() const { return static_cast<bool>(_triggerEnd); }

void Detector::enableTrigger(const Core::TimeSpan &duration) {
  _triggerDuration = duration;
}

void Detector::disableTrigger() { _triggerDuration = boost::none; }

void Detector::setTriggerThresholds(double triggerOn, double triggerOff) {
  _thresTriggerOn = triggerOn;
  _linker.setThresAssociation(_thresTriggerOn);

  if (_thresTriggerOn && config::validateXCorrThreshold(triggerOff)) {
    _thresTriggerOff = triggerOff;
  }
}

void Detector::setArrivalOffsetThreshold(const boost::optional<double> &thres) {
  _linker.setThresArrivalOffset(thres);
}

boost::optional<double> Detector::arrivalOffsetThreshold() const {
  return _linker.thresArrivalOffset();
}

void Detector::setMinArrivals(const boost::optional<size_t> &n) {
  _linker.setMinArrivals(n);
}

boost::optional<size_t> Detector::minArrivals() const {
  return _linker.minArrivals();
}

void Detector::setMergingStrategy(
    linker::MergingStrategy::Type mergingStrategyTypeId) {
  _linker.setMergingStrategy(mergingStrategyTypeId);
}

void Detector::setMaxLatency(const boost::optional<Core::TimeSpan> &latency) {
  _maxLatency = latency;
}

boost::optional<Core::TimeSpan> Detector::maxLatency() const {
  return _maxLatency;
}

size_t Detector::getProcessorCount() const { return _processors.size(); }

void Detector::add(std::unique_ptr<TemplateWaveformProcessor> &&proc,
                   const std::string &waveformStreamId, const Arrival &arrival,
                   const Detector::SensorLocation &loc,
                   const boost::optional<double> &mergingThreshold) {
  proc->setResultCallback(
      [this](const detect::WaveformProcessor *proc, const Record *rec,
             const detect::WaveformProcessor::ResultCPtr &res) {
        storeTemplateResult(
            dynamic_cast<const TemplateWaveformProcessor *>(proc), rec,
            boost::dynamic_pointer_cast<
                const TemplateWaveformProcessor::MatchResult>(res));
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
  ProcessorState p{loc, Core::TimeWindow{}, std::move(proc)};
  _processors.emplace(procId, std::move(p));

  _processorIdx.emplace(waveformStreamId, procId);
}

void Detector::remove(const std::string &waveformStreamId) {
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

void Detector::process(const Record *record) {
  if (status() == Status::kTerminated) {
    throw BaseException{"error while processing: status=" +
                        std::to_string(util::asInteger(Status::kTerminated))};
  }

  if (!_processors.empty()) {
    if (!hasAcceptableLatency(record)) {
      // nothing to do
      return;
    }

    // feed data to template processors
    if (!feed(record)) {
      throw ProcessingError{"error while feeding data to template processors"};
    }

    // XXX(damb): A side-note on trigger facilities when it comes to the
    // linker:
    // - The linker processes only those template results which are fed to the
    // linker i.e. the linker is not aware of the the fact whether a template
    // waveform processor is enabled or disabled, respectively.
    // - Thus, the detector processor can force the linker into a *triggered
    // state* by only feeding data to those processors which are part of the
    // triggering event.

    // process linker results
    while (!_resultQueue.empty()) {
      const auto &result{_resultQueue.front()};

      bool updated{false};
      if (result.fit > _thresTriggerOn.value_or(-1)) {
        if (!_currentResult ||
            (triggered() && result.fit > _currentResult.value().fit &&
             result.getArrivalCount() >=
                 _currentResult.value().getArrivalCount())) {
          _currentResult = result;
          _triggerProcId = result.refProcId;

          updated = true;
        }
      }

      bool withTrigger{_triggerProcId &&
                       _processors.find(*_triggerProcId) != _processors.end() &&
                       _triggerDuration &&
                       *_triggerDuration > Core::TimeSpan{0.0}};

      // enable trigger
      if (withTrigger) {
        const auto &endtime{_processors.at(_triggerProcId.value())
                                .processor->processed()
                                .endTime()};

        if (!triggered() &&
            _currentResult.value().fit >= _thresTriggerOn.value_or(-1)) {
          _triggerEnd = endtime + *_triggerDuration;
          SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggering) %s",
                                       result.debugString().c_str());

          // disable those processors not contributing to the triggering event
          std::vector<std::string> contributing;
          std::transform(
              std::begin(result.results), std::end(result.results),
              std::back_inserter(contributing),
              [](const linker::Association::TemplateResults::value_type &p) {
                return p.first;
              });
          std::for_each(std::begin(_processors), std::end(_processors),
                        [](ProcessorStates::value_type &p) {
                          p.second.processor->disable();
                        });
          std::for_each(std::begin(contributing), std::end(contributing),
                        [this](const std::string &proc_id) {
                          _processors.at(proc_id).processor->enable();
                        });

        } else if (triggered() && updated) {
          SCDETECT_LOG_DEBUG_PROCESSOR(
              this, "Detector result (triggered, updating) %s",
              result.debugString().c_str());
        }
      } else {
        SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result %s",
                                     result.debugString().c_str());
      }

      if (triggered()) {
        if (!updated && result.fit <= _currentResult.value().fit &&
            result.fit >= _thresTriggerOff.value_or(1)) {
          SCDETECT_LOG_DEBUG_PROCESSOR(
              this, "Detector result (triggered, dropped) %s",
              result.debugString().c_str());
        }

        const auto &endtime{
            _processors.at(*_triggerProcId).processor->processed().endTime()};
        // disable trigger if required
        if (endtime > *_triggerEnd ||
            result.fit < _thresTriggerOff.value_or(1) ||
            (util::almostEqual(_currentResult.value().fit, 1.0, 0.000001) &&
             _currentResult.value().getArrivalCount() == getProcessorCount())) {
          resetTrigger();
        }
      }

      // emit detection
      if (!triggered()) {
        Result prepared;
        prepareResult(*_currentResult, prepared);
        emitResult(prepared);
      }

      _resultQueue.pop_front();
    }

    // overall processed endtime
    Core::TimeWindow processed;
    for (const auto &procPair : _processors) {
      const auto procProcessed{procPair.second.processor->processed()};
      if (!procProcessed) {
        processed.setLength(0);
        break;
      } else {
        if (_processed && _processed.endTime() < procProcessed.endTime()) {
          processed = processed | procProcessed;
        } else {
          processed = procProcessed;
        }
      }
    }
    _processed = processed;

    if (!triggered()) {
      resetProcessing();
    }
  }
}

void Detector::reset() {
  _linker.reset();
  resetProcessors();
  resetProcessing();

  _status = Status::kWaitingForData;
}

void Detector::terminate() {
  _linker.terminate();

  if (triggered()) {
    while (!_resultQueue.empty()) {
      const auto &result{_resultQueue.front()};

      if (result.fit > _currentResult.value().fit &&
          result.getArrivalCount() >=
              _currentResult.value().getArrivalCount()) {
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

void Detector::setResultCallback(const PublishResultCallback &callback) {
  _resultCallback = callback;
}

bool Detector::hasAcceptableLatency(const Record *record) {
  if (_maxLatency) {
    return record->endTime() > Core::Time::GMT() - *_maxLatency;
  }

  return true;
}

bool Detector::feed(const Record *record) {
  auto range{_processorIdx.equal_range(record->streamID())};
  for (auto rit{range.first}; rit != range.second; ++rit) {
    const auto &procId{rit->second};
    auto &procState{_processors.at(procId)};

    if (!procState.processor->feed(record)) {
      const auto &status{procState.processor->status()};
      const auto &statusValue{procState.processor->statusValue()};
      SCDETECT_LOG_ERROR_TAGGED(procState.processor->id(),
                                "%s: failed to feed data (tw.start=%s, "
                                "tw.end=%s) to processor. Reason: status=%d, "
                                "statusValue=%f",
                                record->streamID().c_str(),
                                record->startTime().iso().c_str(),
                                record->endTime().iso().c_str(),
                                util::asInteger(status), statusValue);

      return false;
    }

    if (!procState.dataTimeWindowFed) {
      procState.dataTimeWindowFed.setStartTime(record->startTime());
    }
    procState.dataTimeWindowFed.setEndTime(record->endTime());

    if (procState.processor->finished()) {
      return false;
    }
  }

  return true;
}

void Detector::prepareResult(const linker::Association &linkerResult,
                             Detector::Result &result) const {
  const auto &refResult{linkerResult.results.at(linkerResult.refProcId)};
  if (!refResult.matchResult) {
    throw ProcessingError{
        "Failed to prepare result. Reason: missing reference "
        "processor match result"};
  }

  const auto refMatchResult{refResult.matchResult};
  const auto &refStartTime{refMatchResult->timeWindow.startTime()};
  const auto refPickOffset{
      static_cast<double>(refResult.arrival.pick.time - refStartTime)};

  std::unordered_set<std::string> usedChas;
  std::unordered_set<std::string> usedStas;
  // pick offsets (detected arrivals)
  std::vector<double> pickOffsets;

  // TODO(damb): Compute the uncertainty of the arrival time
  // w.r.t.:
  // - the alignment offset of the original waveforms
  //
  // TODO(damb): Compute the uncertainty of the origin time under consideration
  // of the:
  // - the arrival offsets w.r.t. the reference arrival
  const auto &refWaveformStreamId{refResult.arrival.pick.waveformStreamId};
  const auto potOffsets{linkerResult.pot.getOffsets(refWaveformStreamId)};
  Detector::Result::TemplateResults templateResults;
  for (const auto &resPair : linkerResult.results) {
    const auto &procId{resPair.first};
    const auto &templateResult{resPair.second};
    const auto &proc{_processors.at(procId)};

    if (templateResult.matchResult) {
      // compute alignment correction using the POT offset (required, since
      // traces to be cross-correlated cannot be guaranteed to be aligned to
      // sub-sampling interval accuracy)
      const auto &streamId{templateResult.arrival.pick.waveformStreamId};
      const auto n{std::find_if(std::begin(potOffsets), std::end(potOffsets),
                                [&streamId](const PickOffsetNode &n) {
                                  return n.waveformStreamId == streamId;
                                })};
      if (n == potOffsets.end()) {
        throw ProcessingError{
            "Failed to prepare result. Reason: failed to lookup "
            "offset from POT"};
      }

      const auto matchResult{templateResult.matchResult};
      const auto &startTime{matchResult->timeWindow.startTime()};
      const auto &alignmentCorrection{
          refStartTime + Core::TimeSpan{n->pickOffset} - startTime};

      pickOffsets.push_back(
          static_cast<double>(templateResult.arrival.pick.time - startTime) -
          alignmentCorrection - refPickOffset);

      templateResults.emplace(templateResult.arrival.pick.waveformStreamId,
                              Detector::Result::TemplateResult{
                                  templateResult.arrival, proc.sensorLocation,
                                  proc.processor->templateDuration()});
      usedChas.emplace(templateResult.arrival.pick.waveformStreamId);
      usedStas.emplace(proc.sensorLocation.stationId);
    }
  }

  // compute origin time
  const auto &pickOffsetCorrection{
      util::cma(pickOffsets.data(), pickOffsets.size())};
  const auto &refOriginPickOffset{refResult.arrival.pick.offset};
  result.originTime = refStartTime + Core::TimeSpan{refPickOffset} -
                      refOriginPickOffset +
                      Core::TimeSpan{pickOffsetCorrection};
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
  result.numChannelsAssociated = _linker.getAssociatedChannelCount();
  result.numStationsAssociated = associatedStations.size();
}

void Detector::resetProcessing() {
  _currentResult = boost::none;
  // enable processors
  for (auto &procPair : _processors) {
    procPair.second.processor->enable();
  }

  resetTrigger();
}

void Detector::resetTrigger() {
  _triggerProcId = boost::none;
  _triggerEnd = boost::none;
}

void Detector::resetProcessors() {
  std::for_each(std::begin(_processors), std::end(_processors),
                [](ProcessorStates::value_type &p) {
                  p.second.processor->reset();
                  p.second.dataTimeWindowFed = Core::TimeWindow{};
                });
}

void Detector::emitResult(const Detector::Result &result) {
  if (_resultCallback) {
    _resultCallback.value()(result);
  }
}

void Detector::storeTemplateResult(
    const TemplateWaveformProcessor *processor, const Record *record,
    const TemplateWaveformProcessor::MatchResultCPtr &result) {
  if (!processor || !record || !result) {
    return;
  }

  auto &p{_processors.at(processor->id())};
  if (p.processor->finished()) {
    const auto &status{p.processor->status()};
    const auto &statusValue{p.processor->statusValue()};
    auto msg{Core::stringify(
        "Failed to match template (proc_id=%s). Reason: "
        "status=%d, statusValue=%f",
        p.processor->id().c_str(), util::asInteger(status), statusValue)};

    throw TemplateMatchingError{msg};
  }

  _linker.feed(processor, result);
}

void Detector::storeLinkerResult(const linker::Association &linkerResult) {
  _resultQueue.emplace_back(linkerResult);
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
