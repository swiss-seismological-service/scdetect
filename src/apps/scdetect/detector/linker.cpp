#include "linker.h"

#include <iterator>
#include <unordered_set>

#include <boost/functional/hash.hpp>

#include "../template.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace detector {

Linker::Linker(const Core::TimeSpan &on_hold, double arrival_offset_thres)
    : thres_arrival_offset_{arrival_offset_thres}, on_hold_{on_hold} {}

Linker::~Linker() {}

void Linker::set_thres_arrival_offset(const boost::optional<double> &thres) {
  thres_arrival_offset_ = thres;
}

boost::optional<double> Linker::thres_arrival_offset() const {
  return thres_arrival_offset_;
}

void Linker::set_thres_result(const boost::optional<double> &thres) {
  thres_result_ = thres;
}

boost::optional<double> Linker::thres_result() const { return thres_result_; }

void Linker::set_min_arrivals(const boost::optional<size_t> &n) {
  auto v{n};
  if (v && 1 > *v) {
    v = boost::none;
  }

  min_arrivals_ = v;
}

boost::optional<size_t> Linker::min_arrivals() const { return min_arrivals_; }

void Linker::set_on_hold(const Core::TimeSpan &duration) {
  on_hold_ = duration;
}

Core::TimeSpan Linker::on_hold() const { return on_hold_; }

Linker::Status Linker::status() const { return status_; }

size_t Linker::GetAssociatedChannelCount() const {
  std::unordered_set<std::string> wf_ids;
  for (const auto &proc_pair : processors_) {
    wf_ids.emplace(proc_pair.second.arrival.pick.waveform_id);
  }

  return wf_ids.size();
}

size_t Linker::GetProcessorCount() const { return processors_.size(); }

void Linker::Register(const detect::WaveformProcessor *proc,
                      const Arrival &arrival,
                      const Core::TimeSpan &pick_offset) {
  if (proc) {
    processors_.emplace(proc->id(), Processor{arrival, pick_offset});
    pot_valid_ = false;
  }
}

void Linker::Remove(const std::string &proc_id) {
  processors_.erase(proc_id);
  pot_valid_ = false;
}

void Linker::Reset() {
  queue_.clear();
  pot_valid_ = false;

  status_ = Status::kWaitingForData;
}

void Linker::Terminate() {
  // flush pending events
  while (!queue_.empty()) {
    const auto event{queue_.front()};
    if (event.GetArrivalCount() >=
            min_arrivals_.value_or(GetProcessorCount()) &&
        (!thres_result_ || event.result.fit >= *thres_result_)) {
      EmitResult(event.result);
    }

    queue_.pop_front();
  }
  status_ = Status::kTerminated;
}

void Linker::Feed(const detect::WaveformProcessor *proc,
                  const detect::WaveformProcessor::ResultCPtr &res) {
  if (!proc || !res) {
    return;
  }

  if (status() < Status::kTerminated) {
    auto it{processors_.find(proc->id())};
    if (it == processors_.end()) {
      return;
    }

    const auto &match_result{
        boost::dynamic_pointer_cast<const Template::MatchResult>(res)};
    auto &linker_proc{it->second};
    // create a new arrival from a *template arrival*
    auto new_arrival{linker_proc.arrival};
    const auto time{match_result->time_window.startTime() +
                    Core::TimeSpan{match_result->lag} +
                    linker_proc.pick_offset};
    new_arrival.pick.time = time;

    Process(proc, Result::TemplateResult{new_arrival, match_result});
  }
}

void Linker::set_result_callback(const PublishResultCallback &cb) {
  result_callback_ = cb;
}

void Linker::Process(const detect::WaveformProcessor *proc,
                     const Result::TemplateResult &res) {
  if (!processors_.empty()) {
    // update POT
    if (!pot_valid_) {
      CreatePOT();
    }
    pot_.Enable();

    const auto &proc_id{proc->id()};
    const auto &match_result{res.match_result};
    // merge result into existing events
    for (auto event_it = std::begin(queue_); event_it != std::end(queue_);
         ++event_it) {

      if (event_it->GetArrivalCount() < GetProcessorCount()) {
        auto &templ_results{event_it->result.results};
        auto it{templ_results.find(proc_id)};
        if (it == templ_results.end() ||
            match_result->coefficient > it->second.match_result->coefficient) {

          std::vector<Arrival> arrivals{res.arrival};
          std::unordered_set<std::string> wf_ids;
          for (const auto &templ_res_pair : templ_results) {
            const auto &a{templ_res_pair.second.arrival};
            arrivals.push_back(a);
            wf_ids.emplace(a.pick.waveform_id);
          }

          POT pot{arrivals};

          if (thres_arrival_offset_) {
            // prepare reference POT
            pot_.Disable(wf_ids);

            std::unordered_set<std::string> exceeded;
            if (!ValidatePickOffsets(pot_, pot, exceeded,
                                     *thres_arrival_offset_) ||
                !exceeded.empty()) {
              continue;
            }
          }

          event_it->MergeResult(proc_id, res, pot);
        }
        pot_.Enable();
      }
    }

    const auto now{Core::Time::GMT()};
    // create new event
    Event event{now + on_hold_};
    event.MergeResult(proc_id, res, POT{std::vector<Arrival>{res.arrival}});
    queue_.emplace_back(event);

    std::vector<EventQueue::iterator> ready;
    for (auto it = std::begin(queue_); it != std::end(queue_); ++it) {
      const auto arrival_count{it->GetArrivalCount()};
      // emit results which are ready and surpass threshold
      if (arrival_count == GetProcessorCount() ||
          (now >= it->expired &&
           arrival_count >= min_arrivals_.value_or(GetProcessorCount()))) {

        if (!thres_result_ || it->result.fit >= *thres_result_) {
          EmitResult(it->result);
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
      queue_.erase(it);
    }
  }
}

void Linker::EmitResult(const Result &res) {
  if (result_callback_) {
    result_callback_.value()(res);
  }
}

void Linker::CreatePOT() {
  std::vector<Arrival> arrivals;
  using pair_type = Processors::value_type;
  std::transform(processors_.cbegin(), processors_.cend(),
                 back_inserter(arrivals),
                 [](const pair_type &p) { return p.second.arrival; });

  // XXX(damb): The current implementation simply recreates the POT
  pot_ = POT(arrivals);
  pot_valid_ = true;
}

/* ------------------------------------------------------------------------- */
size_t Linker::Result::GetArrivalCount() const { return results.size(); }

std::string Linker::Result::DebugString() const {
  const Core::Time starttime{
      results.at(ref_proc_id).match_result->time_window.startTime()};
  const Core::Time endtime{starttime +
                           Core::TimeSpan{pot.pick_offset().value_or(0)}};
  return std::string{"(" + starttime.iso() + " - " + endtime.iso() +
                     "): fit=" + std::to_string(fit) +
                     ", arrival_count=" + std::to_string(GetArrivalCount())};
}

/* ------------------------------------------------------------------------- */
void Linker::Event::MergeResult(const std::string &proc_id,
                                const Result::TemplateResult &res,
                                const POT &pot) {

  auto &templ_results{result.results};
  templ_results.emplace(proc_id, res);

  std::vector<double> fits;
  std::transform(std::begin(templ_results), std::end(templ_results),
                 std::back_inserter(fits),
                 [](const Result::TemplateResults::value_type &p) {
                   return p.second.match_result->coefficient;
                 });

  // XXX(damb): Currently, we use the mean in order to compute the overall
  // event's score
  result.fit = utils::CMA(fits.data(), fits.size());
  result.pot = pot;
  if (!ref_pick_time || res.arrival.pick.time < ref_pick_time) {
    ref_pick_time = res.arrival.pick.time;
    result.ref_proc_id = proc_id;
  }
}

size_t Linker::Event::GetArrivalCount() const { return result.results.size(); }

} // namespace detector
} // namespace detect
} // namespace Seiscomp

namespace std {

inline std::size_t
hash<Seiscomp::detect::detector::Linker::Result::TemplateResult>::operator()(
    const Seiscomp::detect::detector::Linker::Result::TemplateResult &tr)
    const noexcept {

  std::size_t ret{0};
  boost::hash_combine(
      ret, std::hash<Seiscomp::detect::detector::Arrival>{}(tr.arrival));

  if (tr.match_result) {
    boost::hash_combine(ret, std::hash<double>{}(tr.match_result->coefficient));
  }

  return ret;
}

} // namespace std
