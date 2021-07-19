#include "linker.h"

#include <boost/functional/hash.hpp>
#include <iterator>
#include <unordered_set>

#include "../utils.h"

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
      const auto time{res->timeWindow.startTime() + Core::TimeSpan{res->lag} +
                      pickOffset};
      newArrival.pick.time = time;

      linker::Association::TemplateResult templateResult{newArrival, res};
      // filter/drop based on merging strategy
      if (_mergingStrategy && _thresAssociation &&
          !_mergingStrategy->operator()(
              templateResult, *_thresAssociation,
              linkerProc.mergingThreshold.value_or(*_thresAssociation))) {
#ifdef SCDETECT_DEBUG
        SCDETECT_LOG_DEBUG_PROCESSOR(proc,
                                     "Dropping result due to merging strategy "
                                     "applied: fit=%9f, lag=%10f",
                                     templateResult.matchResult->coefficient,
                                     templateResult.matchResult->lag);
#endif
        return;
      }

      process(proc, templateResult);
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
    const auto &matchResult{res.matchResult};
    // merge result into existing events
    for (auto eventIt = std::begin(_queue); eventIt != std::end(_queue);
         ++eventIt) {
      if (eventIt->getArrivalCount() < getProcessorCount()) {
        auto &templResults{eventIt->association.results};
        auto it{templResults.find(procId)};
        if (it == templResults.end() ||
            matchResult->coefficient > it->second.matchResult->coefficient) {
          std::vector<Arrival> arrivals{res.arrival};
          std::unordered_set<std::string> wfIds;
          for (const auto &templResultPair : templResults) {
            const auto &a{templResultPair.second.arrival};
            arrivals.push_back(a);
            wfIds.emplace(a.pick.waveformStreamId);
          }

          POT pot{arrivals};

          if (_thresArrivalOffset) {
            // prepare reference POT
            _pot.disable(wfIds);

            std::unordered_set<std::string> exceeded;
            if (!validatePickOffsets(_pot, pot, exceeded,
                                     *_thresArrivalOffset) ||
                !exceeded.empty()) {
              continue;
            }
          }

          eventIt->feed(procId, res, pot);
        }
        _pot.enable();
      }
    }

    const auto now{Core::Time::GMT()};
    // create new event
    Event event{now + _onHold};
    event.feed(procId, res, POT{std::vector<Arrival>{res.arrival}});
    _queue.emplace_back(event);

    std::vector<EventQueue::iterator> ready;
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
  std::vector<Arrival> arrivals;
  using pair_type = Processors::value_type;
  std::transform(_processors.cbegin(), _processors.cend(),
                 back_inserter(arrivals),
                 [](const pair_type &p) { return p.second.arrival; });

  // XXX(damb): The current implementation simply recreates the POT
  _pot = POT(arrivals);
  _potValid = true;
}

/* ------------------------------------------------------------------------- */
Linker::Event::Event(const Core::Time &expired) : expired{expired} {}

void Linker::Event::feed(const std::string &procId,
                         const linker::Association::TemplateResult &res,
                         const POT &pot) {
  auto &templateResults{association.results};
  templateResults.emplace(procId, res);

  std::vector<double> fits;
  std::transform(std::begin(templateResults), std::end(templateResults),
                 std::back_inserter(fits),
                 [](const linker::Association::TemplateResults::value_type &p) {
                   return p.second.matchResult->coefficient;
                 });

  // compute the overall event's score
  association.fit = utils::cma(fits.data(), fits.size());
  association.pot = pot;
  if (!refPickTime || res.arrival.pick.time < refPickTime) {
    refPickTime = res.arrival.pick.time;
    association.refProcId = procId;
  }
}

size_t Linker::Event::getArrivalCount() const {
  return association.results.size();
}

bool Linker::Event::isExpired(const Core::Time &now) const {
  return now >= expired;
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
