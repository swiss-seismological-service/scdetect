#include "linker.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <memory>
#include <unordered_set>

#include "../util/math.h"
#include "../util/util.h"
#include "detail.h"

namespace Seiscomp {
namespace detect {
namespace detector {

Linker::Linker(const Core::TimeSpan &onHold,
               const Core::TimeSpan &arrivalOffsetThres)
    : _thresArrivalOffset{arrivalOffsetThres}, _onHold{onHold} {}

void Linker::setThresArrivalOffset(
    const boost::optional<Core::TimeSpan> &thres) {
  _thresArrivalOffset = thres;
}

boost::optional<Core::TimeSpan> Linker::thresArrivalOffset() const {
  return _thresArrivalOffset;
}

void Linker::setThresAssociation(const boost::optional<double> &thres) {
  _thresAssociation = thres;
}

boost::optional<double> Linker::thresAssociation() const {
  return _thresAssociation;
}

void Linker::setMinArrivals(const boost::optional<size_t> &n) {
  auto v{n};
  if (v && 1 > *v) {
    v = boost::none;
  }

  _minArrivals = v;
}

boost::optional<size_t> Linker::minArrivals() const { return _minArrivals; }

void Linker::setOnHold(const Core::TimeSpan &duration) { _onHold = duration; }

Core::TimeSpan Linker::onHold() const { return _onHold; }

void Linker::setMergingStrategy(MergingStrategy mergingStrategy) {
  _mergingStrategy = std::move(mergingStrategy);
}

size_t Linker::channelCount() const {
  std::unordered_set<std::string> wfIds;
  for (const auto &procPair : _processors) {
    wfIds.emplace(procPair.second.arrival.pick.waveformStreamId);
  }

  return wfIds.size();
}

size_t Linker::processorCount() const { return _processors.size(); }

void Linker::add(const TemplateWaveformProcessor *proc, const Arrival &arrival,
                 const boost::optional<double> &mergingThreshold) {
  if (proc) {
    _processors.emplace(proc->id(), Processor{proc, arrival, mergingThreshold});
    _potValid = false;
  }
}

void Linker::remove(const std::string &procId) {
  _processors.erase(procId);
  _potValid = false;
}

void Linker::reset() {
  _queue.clear();
  _potValid = false;
}

void Linker::flush() {
  // flush pending events
  while (!_queue.empty()) {
    const auto event{_queue.front()};
    if (event.associatedProcessorCount() >=
            _minArrivals.value_or(processorCount()) &&
        (!_thresAssociation || event.association.score >= *_thresAssociation)) {
      emitResult(event.association);
    }

    _queue.pop_front();
  }
}

void Linker::feed(
    const TemplateWaveformProcessor *proc,
    std::unique_ptr<const TemplateWaveformProcessor::MatchResult> matchResult) {
  assert((proc && matchResult));

  auto it{_processors.find(proc->id())};
  if (it == _processors.end()) {
    return;
  }

  auto &linkerProc{it->second};
  // create a new arrival from a *template arrival*
  auto newArrival{linkerProc.arrival};

  std::shared_ptr<const TemplateWaveformProcessor::MatchResult> result{
      std::move(matchResult)};
  // XXX(damb): recompute the pickOffset; the template proc might have
  // changed the underlying template waveform (due to resampling)
  const auto currentPickOffset{linkerProc.arrival.pick.time -
                               linkerProc.proc->templateWaveform().startTime()};
  for (auto valueIt{result->localMaxima.begin()};
       valueIt != result->localMaxima.end(); ++valueIt) {
    const auto time{result->timeWindow.startTime() + valueIt->lag +
                    currentPickOffset};
    newArrival.pick.time = time;

    linker::Association::TemplateResult templateResult{newArrival, valueIt,
                                                       result};
    // filter/drop based on merging strategy
    if (_thresAssociation &&
        !_mergingStrategy(
            templateResult, *_thresAssociation,
            linkerProc.mergingThreshold.value_or(*_thresAssociation))) {
#ifdef SCDETECT_DEBUG
      SCDETECT_LOG_DEBUG_PROCESSOR(
          proc,
          "[%s] [%s - %s] Dropping result due to merging "
          "strategy applied: time=%s, score=%9f, lag=%10f",
          newArrival.pick.waveformStreamId.c_str(),
          result->timeWindow.startTime().iso().c_str(),
          result->timeWindow.endTime().iso().c_str(), time.iso().c_str(),
          valueIt->coefficient, static_cast<double>(valueIt->lag));
#endif
      continue;
    }

#ifdef SCDETECT_DEBUG
    SCDETECT_LOG_DEBUG_PROCESSOR(
        proc,
        "[%s] [%s - %s] Trying to merge result: time=%s, score=%9f, lag=%10f",
        newArrival.pick.waveformStreamId.c_str(),
        result->timeWindow.startTime().iso().c_str(),
        result->timeWindow.endTime().iso().c_str(), time.iso().c_str(),
        valueIt->coefficient, static_cast<double>(valueIt->lag));
#endif
    process(proc, templateResult);
  }
}

void Linker::setResultCallback(const PublishResultCallback &callback) {
  _resultCallback = callback;
}

void Linker::process(const TemplateWaveformProcessor *proc,
                     const linker::Association::TemplateResult &result) {
  if (_processors.empty()) {
    return;
  }

  // update POT
  if (!_potValid) {
    createPot();
  }

  const auto &procId{proc->id()};
  auto resultIt{result.resultIt};
  // merge result into existing candidates
  for (auto candidateIt = std::begin(_queue); candidateIt != std::end(_queue);
       ++candidateIt) {
    if (candidateIt->associatedProcessorCount() < processorCount()) {
      auto &candidateTemplateResults{candidateIt->association.results};
      auto it{candidateTemplateResults.find(procId)};

      bool newPick{it == candidateTemplateResults.end()};
      if (newPick || resultIt->coefficient > it->second.resultIt->coefficient) {
        if (_thresArrivalOffset) {
          auto candidatePOTData{
              createCandidatePOTData(*candidateIt, procId, result)};
          if (!_pot.validateEnabledOffsets(procId, candidatePOTData.offsets,
                                           candidatePOTData.mask,
                                           *_thresArrivalOffset)) {
            continue;
          }
        }
        candidateIt->feed(procId, result);
      }
    }
  }

  const auto now{Core::Time::GMT()};
  // create new candidate association
  Candidate newCandidate{now + _onHold};
  newCandidate.feed(procId, result);
  _queue.emplace_back(newCandidate);

  std::vector<CandidateQueue::iterator> ready;
  for (auto it = std::begin(_queue); it != std::end(_queue); ++it) {
    const auto arrivalCount{it->associatedProcessorCount()};
    // emit results which are ready and surpass threshold
    if (arrivalCount == processorCount() ||
        (now >= it->expired &&
         arrivalCount >= _minArrivals.value_or(processorCount()))) {
      if (!_thresAssociation || it->association.score >= *_thresAssociation) {
        emitResult(it->association);
      }
      ready.push_back(it);
    }
    // drop expired result
    else if (now >= it->expired) {
      ready.push_back(it);
    }
  }

  // clean up result queue
  for (auto &it : ready) {
    _queue.erase(it);
  }
}

void Linker::emitResult(const linker::Association &result) {
  if (_resultCallback) {
    _resultCallback.value()(result);
  }
}

void Linker::createPot() {
  std::vector<linker::POT::Entry> entries;
  using pair_type = Processors::value_type;
  std::transform(_processors.cbegin(), _processors.cend(),
                 back_inserter(entries), [](const pair_type &p) {
                   return linker::POT::Entry{p.second.arrival.pick.time,
                                             p.second.proc->id(), true};
                 });

  // XXX(damb): the current implementation simply recreates the POT
  _pot = linker::POT(entries);
  _potValid = true;
}

Linker::CandidatePOTData Linker::createCandidatePOTData(
    const Candidate &candidate, const std::string &processorId,
    const linker::Association::TemplateResult &newResult) {
  auto allProcessorIds{_pot.processorIds()};
  const auto &associatedCandidateTemplateResults{candidate.association.results};

  auto associatedProcessorIds{
      util::map_keys(associatedCandidateTemplateResults)};
  std::set<detail::ProcessorIdType> enabledProcessorIds{
      associatedProcessorIds.begin(), associatedProcessorIds.end()};
  enabledProcessorIds.emplace(processorId);

  CandidatePOTData ret(allProcessorIds.size());
  for (std::size_t i{0}; i < allProcessorIds.size(); ++i) {
    const auto &curProcessorId{allProcessorIds[i]};
    bool disabled{enabledProcessorIds.find(curProcessorId) ==
                  enabledProcessorIds.end()};
    if (disabled) {
      continue;
    }

    if (curProcessorId != processorId) {
      ret.offsets[i] = std::abs(static_cast<double>(
          associatedCandidateTemplateResults.at(curProcessorId)
              .arrival.pick.time -
          newResult.arrival.pick.time));
    } else {
      ret.offsets[i] = 0;
    }
    ret.mask[i] = true;
  }

  return ret;
}

/* ------------------------------------------------------------------------- */
Linker::Candidate::Candidate(const Core::Time &expired) : expired{expired} {}

void Linker::Candidate::feed(const std::string &procId,
                             const linker::Association::TemplateResult &res) {
  auto &templateResults{association.results};
  templateResults.emplace(procId, res);

  std::vector<double> scores;
  std::transform(std::begin(templateResults), std::end(templateResults),
                 std::back_inserter(scores),
                 [](const linker::Association::TemplateResults::value_type &p) {
                   return p.second.resultIt->coefficient;
                 });

  // compute the overall event's score
  association.score = util::cma(scores.data(), scores.size());
}

size_t Linker::Candidate::associatedProcessorCount() const {
  return association.processorCount();
}

bool Linker::Candidate::isExpired(const Core::Time &now) const {
  return now >= expired;
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
