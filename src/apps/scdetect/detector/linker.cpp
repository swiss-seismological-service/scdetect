#include "linker.h"

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <iterator>
#include <unordered_set>

#include "../util/math.h"

namespace Seiscomp {
namespace detect {
namespace detector {

Linker::Linker(const Core::TimeSpan &onHold, double arrivalOffsetThres)
    : _thresArrivalOffset{arrivalOffsetThres}, _onHold{onHold} {}

Linker::~Linker() {}

void Linker::setThresArrivalOffset(const boost::optional<double> &thres) {
  _thresArrivalOffset = thres;
}

boost::optional<double> Linker::thresArrivalOffset() const {
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

void Linker::setMergingStrategy(
    linker::MergingStrategy::Type mergingStrategyTypeId) {
  _mergingStrategy = linker::MergingStrategy::Create(mergingStrategyTypeId);
}

Linker::Status Linker::status() const { return _status; }

size_t Linker::getAssociatedChannelCount() const {
  std::unordered_set<std::string> wfIds;
  for (const auto &procPair : _processors) {
    wfIds.emplace(procPair.second.arrival.pick.waveformStreamId);
  }

  return wfIds.size();
}

size_t Linker::getProcessorCount() const { return _processors.size(); }

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

  _status = Status::kWaitingForData;
}

void Linker::terminate() {
  // flush pending events
  while (!_queue.empty()) {
    const auto event{_queue.front()};
    if (event.getArrivalCount() >= _minArrivals.value_or(getProcessorCount()) &&
        (!_thresAssociation || event.association.fit >= *_thresAssociation)) {
      emitResult(event.association);
    }

    _queue.pop_front();
  }
  _status = Status::kTerminated;
}

void Linker::feed(const TemplateWaveformProcessor *proc,
                  const TemplateWaveformProcessor::MatchResultCPtr &res) {
  if (!proc || !res) {
    return;
  }

  if (status() < Status::kTerminated) {
    auto it{_processors.find(proc->id())};
    if (it == _processors.end()) {
      return;
    }

    auto &linkerProc{it->second};
    // create a new arrival from a *template arrival*
    auto newArrival{linkerProc.arrival};

    const auto templateStartTime{linkerProc.proc->templateStartTime()};
    if (templateStartTime) {
      // XXX(damb): recompute the pickOffset; the template proc might have
      // changed the underlying template waveform (due to resampling)
      const auto pickOffset{linkerProc.arrival.pick.time - *templateStartTime};
      for (auto valueIt{res->localMaxima.begin()};
           valueIt != res->localMaxima.end(); ++valueIt) {
        const auto time{res->timeWindow.startTime() +
                        Core::TimeSpan{valueIt->lag} + pickOffset};
        newArrival.pick.time = time;

        linker::Association::TemplateResult templateResult{newArrival, valueIt,
                                                           res};
        // filter/drop based on merging strategy
        if (_mergingStrategy && _thresAssociation &&
            !_mergingStrategy->operator()(
                templateResult, *_thresAssociation,
                linkerProc.mergingThreshold.value_or(*_thresAssociation))) {
#ifdef SCDETECT_DEBUG
          SCDETECT_LOG_DEBUG_PROCESSOR(
              proc,
              "[%s] [%s - %s] Dropping result due to merging "
              "strategy applied: time=%s, fit=%9f, lag=%10f",
              newArrival.pick.waveformStreamId.c_str(),
              res->timeWindow.startTime().iso().c_str(),
              res->timeWindow.endTime().iso().c_str(), time.iso().c_str(),
              valueIt->coefficient, valueIt->lag);
#endif
          continue;
        }

#ifdef SCDETECT_DEBUG
        SCDETECT_LOG_DEBUG_PROCESSOR(
            proc,
            "[%s] [%s - %s] Trying to merge result: time=%s, fit=%9f, lag=%10f",
            newArrival.pick.waveformStreamId.c_str(),
            res->timeWindow.startTime().iso().c_str(),
            res->timeWindow.endTime().iso().c_str(), valueIt->coefficient,
            valueIt->lag, time.iso().c_str());
#endif
        process(proc, templateResult);
      }
    }
  }
}

void Linker::setResultCallback(const PublishResultCallback &callback) {
  _resultCallback = callback;
}

void Linker::process(const TemplateWaveformProcessor *proc,
                     const linker::Association::TemplateResult &res) {
  if (!_processors.empty()) {
    // update POT
    if (!_potValid) {
      createPot();
    }
    _pot.enable();

    const auto &procId{proc->id()};
    auto resultIt{res.resultIt};
    // merge result into existing candidates
    for (auto candidateIt = std::begin(_queue); candidateIt != std::end(_queue);
         ++candidateIt) {
      if (candidateIt->getArrivalCount() < getProcessorCount()) {
        auto &candidateTemplateResults{candidateIt->association.results};
        auto it{candidateTemplateResults.find(procId)};

        bool newPick{it == candidateTemplateResults.end()};
        if (newPick ||
            resultIt->coefficient > it->second.resultIt->coefficient) {
          if (_thresArrivalOffset) {
            auto cmp{createCandidatePOT(*candidateIt, procId, res)};
            if (!_pot.validateEnabledOffsets(cmp, *_thresArrivalOffset)) {
              continue;
            }
          }
          candidateIt->feed(procId, res);
        }
      }
    }

    const auto now{Core::Time::GMT()};
    // create new candidate association
    Candidate newCandidate{now + _onHold};
    newCandidate.feed(procId, res);
    _queue.emplace_back(newCandidate);

    std::vector<CandidateQueue::iterator> ready;
    for (auto it = std::begin(_queue); it != std::end(_queue); ++it) {
      const auto arrivalCount{it->getArrivalCount()};
      // emit results which are ready and surpass threshold
      if (arrivalCount == getProcessorCount() ||
          (now >= it->expired &&
           arrivalCount >= _minArrivals.value_or(getProcessorCount()))) {
        if (!_thresAssociation || it->association.fit >= *_thresAssociation) {
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
}

void Linker::emitResult(const linker::Association &res) {
  if (_resultCallback) {
    _resultCallback.value()(res);
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

  // XXX(damb): The current implementation simply recreates the POT
  _pot = linker::POT(entries);
  _potValid = true;
}

linker::POT Linker::createCandidatePOT(
    const Candidate &candidate, const std::string &processorId,
    const linker::Association::TemplateResult &newResult) {
  std::set<std::string> allProcessorIds;
  for (const auto &processorsPair : _processors) {
    allProcessorIds.emplace(processorsPair.first);
  }
  std::set<std::string> associatedProcessorId{processorId};
  const auto &associatedCandidateTemplateResults{candidate.association.results};
  for (const auto &associatedTemplateResultPair :
       associatedCandidateTemplateResults) {
    associatedProcessorId.emplace(associatedTemplateResultPair.first);
  }
  std::set<std::string> additionalProcessorIds;
  std::set_difference(
      std::begin(allProcessorIds), std::end(allProcessorIds),
      std::begin(associatedProcessorId), std::end(associatedProcessorId),
      std::inserter(additionalProcessorIds, std::end(additionalProcessorIds)));

  std::vector<linker::POT::Entry> candidatePOTEntries{
      {newResult.arrival.pick.time, processorId, true}};
  for (const auto &templateResultPair : associatedCandidateTemplateResults) {
    const auto associatedProcId{templateResultPair.first};
    if (processorId != associatedProcId) {
      const auto &templateResult{templateResultPair.second};
      candidatePOTEntries.push_back(
          {templateResult.arrival.pick.time, associatedProcId, true});
    }
  }

  for (const auto &p : additionalProcessorIds) {
    candidatePOTEntries.push_back({Core::Time{}, p, false});
  }

  return linker::POT(candidatePOTEntries);
}

/* ------------------------------------------------------------------------- */
Linker::Candidate::Candidate(const Core::Time &expired) : expired{expired} {}

void Linker::Candidate::feed(const std::string &procId,
                             const linker::Association::TemplateResult &res) {
  auto &templateResults{association.results};
  templateResults.emplace(procId, res);

  std::vector<double> fits;
  std::transform(std::begin(templateResults), std::end(templateResults),
                 std::back_inserter(fits),
                 [](const linker::Association::TemplateResults::value_type &p) {
                   return p.second.resultIt->coefficient;
                 });

  // compute the overall event's score
  association.fit = util::cma(fits.data(), fits.size());
}

size_t Linker::Candidate::getArrivalCount() const {
  return association.results.size();
}

bool Linker::Candidate::isExpired(const Core::Time &now) const {
  return now >= expired;
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
