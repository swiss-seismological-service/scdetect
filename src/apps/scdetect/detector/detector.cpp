#include "detector.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "seiscomp/core/datetime.h"
#include "seiscomp/core/timewindow.h"
#include "seiscomp/datamodel/origin.h"

#include "../log.h"
#include "../utils.h"
#include "../validators.h"

namespace Seiscomp {
namespace detect {
namespace detector {

Detector::Detector(const detect::Processor *detector,
                   const DataModel::OriginCPtr &origin)
    : Processor{detector->id()}, origin_{origin} {

  linker_.set_result_callback(
      [this](const Linker::Result &res) { return StoreLinkerResult(res); });
}

Detector::~Detector() {}

Detector::BaseException::BaseException()
    : Exception{"base processing strategy exception"} {}

Detector::ProcessingError::ProcessingError()
    : BaseException{"error while processing"} {}

Detector::TemplateMatchingError::TemplateMatchingError()
    : ProcessingError{"error while matching template"} {}

Detector::Status Detector::status() const { return status_; }

const Core::TimeWindow &Detector::processed() const { return processed_; }

bool Detector::triggered() const { return static_cast<bool>(trigger_end_); }

void Detector::EnableTrigger(const Core::TimeSpan &duration) {
  trigger_duration_ = duration;
}

void Detector::DisableTrigger() { trigger_duration_ = boost::none; }

void Detector::set_trigger_thresholds(double trigger_on, double trigger_off) {
  thres_trigger_on_ = trigger_on;
  linker_.set_thres_result(thres_trigger_on_);

  if (thres_trigger_on_ && config::ValidateXCorrThreshold(trigger_off)) {
    thres_trigger_off_ = trigger_off;
  }
}

void Detector::set_arrival_offset_threshold(
    const boost::optional<double> &thres) {
  linker_.set_thres_arrival_offset(thres);
}

boost::optional<double> Detector::arrival_offset_threshold() const {
  return linker_.thres_arrival_offset();
}

void Detector::set_min_arrivals(const boost::optional<size_t> &n) {
  linker_.set_min_arrivals(n);
}

boost::optional<size_t> Detector::min_arrivals() const {
  return linker_.min_arrivals();
}

void Detector::set_maximum_latency(
    const boost::optional<Core::TimeSpan> &latency) {
  max_latency_ = latency;
}

boost::optional<Core::TimeSpan> Detector::maximum_latency() const {
  return max_latency_;
}

size_t Detector::GetProcessorCount() const { return processors_.size(); }

void Detector::Register(std::unique_ptr<detect::WaveformProcessor> &&proc,
                        const std::shared_ptr<const RecordSequence> &buf,
                        const std::string &stream_id, const Arrival &arrival,
                        const Core::TimeSpan &pick_offset,
                        const Detector::SensorLocation &loc) {

  proc->set_result_callback(
      [this](const detect::WaveformProcessor *p, const Record *rec,
             const detect::WaveformProcessor::ResultCPtr &res) {
        StoreTemplateResult(p, rec, res);
      });

  // XXX(damb): Replace the arrival with a *pseudo arrival* i.e. an arrival
  // which is associated with the stream to be processed
  Arrival pseudo_arrival{arrival};
  pseudo_arrival.pick.waveform_id = stream_id;

  linker_.Register(proc.get(), pseudo_arrival, pick_offset);
  const auto on_hold_duration{max_latency_.value_or(0.0) + proc->init_time() +
                              linker_safety_margin_};
  if (linker_.on_hold() < on_hold_duration) {
    linker_.set_on_hold(on_hold_duration);
  }

  const auto proc_id{proc->id()};
  ProcessorState p{std::move(proc), buf, loc};
  processors_.emplace(proc_id, std::move(p));

  processor_idx_.emplace(stream_id, proc_id);
}

void Detector::Remove(const std::string &stream_id) {
  auto range{processor_idx_.equal_range(stream_id)};
  for (auto &rit = range.first; rit != range.second; ++rit) {
    linker_.Remove(rit->first);

    processor_idx_.erase(rit);
    processors_.erase(rit->second);
  }

  // update linker
  using pair_type = ProcessorStates::value_type;
  const auto it{
      std::max_element(std::begin(processors_), std::end(processors_),
                       [](const pair_type &lhs, const pair_type &rhs) {
                         return lhs.second.processor->init_time() <
                                rhs.second.processor->init_time();
                       })};
  if (it == std::end(processors_)) {
    return;
  }

  const auto max_on_hold_duration{it->second.processor->init_time() +
                                  max_latency_.value_or(0.0) +
                                  linker_safety_margin_};
  if (max_on_hold_duration > linker_.on_hold()) {
    linker_.set_on_hold(max_on_hold_duration);
  }
}

void Detector::Process(const std::string &waveform_id_hint) {
  if (status() == Status::kTerminated) {
    throw BaseException{"error while processing: status=" +
                        std::to_string(utils::as_integer(Status::kTerminated))};
  }

  if (!processors_.empty()) {

    TimeWindows tws;
    if (!PrepareProcessing(tws, waveform_id_hint)) {
      // nothing to do
      return;
    }

    // feed data to template processors
    if (!Feed(tws)) {
      throw ProcessingError{"error while feeding data to template processors"};
    }

    // XXX(damb): A side-note on trigger facilities when it comes to the
    // linker:
    // - The linker processes only those template results which are fed to the
    // linker i.e. the linker is not aware of the the fact if a template
    // processor is enabled or disabled, respectively.
    // - Thus, the detector processor can force the linker into a *triggered
    // state* by only feeding data to those processors which are part of the
    // triggering event.

    // process linker results
    while (!result_queue_.empty()) {
      const auto &result{result_queue_.front()};

      bool updated{false};
      if (result.fit > thres_trigger_on_.value_or(-1)) {
        if (!current_result_ ||
            (triggered() && result.fit > current_result_.value().fit &&
             result.GetArrivalCount() >=
                 current_result_.value().GetArrivalCount())) {
          current_result_ = result;
          trigger_proc_id_ = result.ref_proc_id;

          updated = true;
        }
      }

      bool with_trigger{
          trigger_proc_id_ &&
          processors_.find(*trigger_proc_id_) != processors_.end() &&
          trigger_duration_ && *trigger_duration_ > Core::TimeSpan{0.0}};

      // enable trigger
      if (with_trigger) {
        const auto &endtime{processors_.at(trigger_proc_id_.value())
                                .processor->processed()
                                .endTime()};

        if (!triggered() &&
            current_result_.value().fit >= thres_trigger_on_.value_or(-1)) {
          trigger_end_ = endtime + *trigger_duration_;
          SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggering) %s",
                                       result.DebugString().c_str());

          // disable those processors not contributing to the triggering event
          std::vector<std::string> contributing;
          std::transform(
              std::begin(result.results), std::end(result.results),
              std::back_inserter(contributing),
              [](const Linker::Result::TemplateResults::value_type &p) {
                return p.first;
              });
          std::for_each(std::begin(processors_), std::end(processors_),
                        [](ProcessorStates::value_type &p) {
                          p.second.processor->disable();
                        });
          std::for_each(std::begin(contributing), std::end(contributing),
                        [this](const std::string &proc_id) {
                          processors_.at(proc_id).processor->enable();
                        });

        } else if (triggered() && updated) {
          SCDETECT_LOG_DEBUG_PROCESSOR(
              this, "Detector result (triggered, updating) %s",
              result.DebugString().c_str());
        }
      } else {
        SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result %s",
                                     result.DebugString().c_str());
      }

      if (triggered()) {
        if (!updated && result.fit <= current_result_.value().fit &&
            result.fit >= thres_trigger_off_.value_or(1)) {
          SCDETECT_LOG_DEBUG_PROCESSOR(
              this, "Detector result (triggered, dropped) %s",
              result.DebugString().c_str());
        }

        const auto &endtime{
            processors_.at(*trigger_proc_id_).processor->processed().endTime()};
        // disable trigger if required
        if (endtime > *trigger_end_ ||
            result.fit < thres_trigger_off_.value_or(1) ||
            (utils::AlmostEqual(current_result_.value().fit, 1.0, 0.000001) &&
             current_result_.value().GetArrivalCount() ==
                 GetProcessorCount())) {
          ResetTrigger();
        }
      }

      // emit detection
      if (!triggered()) {
        Result prepared;
        PrepareResult(*current_result_, prepared);
        EmitResult(prepared);
      }

      result_queue_.pop_front();
    }

    // overall processed endtime
    Core::TimeWindow processed;
    for (const auto &proc_pair : processors_) {
      const auto proc_processed{proc_pair.second.processor->processed()};
      if (!proc_processed) {
        processed.setLength(0);
        break;
      } else {
        if (processed_ && processed_.endTime() < proc_processed.endTime()) {
          processed = processed | proc_processed;
        } else {
          processed = proc_processed;
        }
      }
    }
    processed_ = processed;

    if (!triggered()) {
      ResetProcessing();
    } else {
      ResetProcessors();
    }
  }
}

void Detector::Reset() {
  linker_.Reset();
  ResetProcessing();

  status_ = Status::kWaitingForData;
}

void Detector::Terminate() {
  linker_.Terminate();

  if (triggered()) {
    while (!result_queue_.empty()) {
      const auto &result{result_queue_.front()};

      if (result.fit > current_result_.value().fit &&
          result.GetArrivalCount() >=
              current_result_.value().GetArrivalCount()) {
        current_result_ = result;
      }

      result_queue_.pop_front();
    }

    Result prepared;
    PrepareResult(*current_result_, prepared);
    EmitResult(prepared);

  } else if (!trigger_duration_) {
    while (!result_queue_.empty()) {
      const auto &result{result_queue_.front()};
      current_result_ = result;
      result_queue_.pop_front();

      Result prepared;
      PrepareResult(*current_result_, prepared);
      EmitResult(prepared);

      current_result_ = boost::none;
    }
  }

  status_ = Status::kTerminated;
}

void Detector::set_result_callback(const PublishResultCallback &cb) {
  result_callback_ = cb;
}

bool Detector::PrepareProcessing(Detector::TimeWindows &tws,
                                 const std::string &waveform_id_hint) {
  Core::Time latency_endtime;
  if (max_latency_) {
    latency_endtime = Core::Time::GMT() - *max_latency_;
  }

  const auto range{processor_idx_.equal_range(waveform_id_hint)};
  for (auto rit = range.first; rit != range.second; ++rit) {
    const auto &proc_id{rit->second};
    const auto &proc{processors_.at(proc_id)};
    if (proc.processor->enabled()) {
      Core::TimeWindow tw{proc.buffer->windows().back()};
      if (!tw.endTime() ||
          tw.endTime() <= proc.processor->processed().endTime()) {
        continue;
      }

      if (tw.startTime() < proc.processor->processed().endTime()) {
        tw.setStartTime(proc.processor->processed().endTime());
      }

      // skip data with too high latency
      if (latency_endtime && (tw.endTime() < latency_endtime)) {
        continue;
      }

      tws.emplace(proc_id, tw);
    }
  }

  return !tws.empty();
}

bool Detector::Feed(const TimeWindows &tws) {
  for (const auto &tws_pair : tws) {
    auto &proc{processors_.at(tws_pair.first)};
    if (!proc.processor->enabled()) {
      continue;
    }

    auto trace{proc.buffer->contiguousRecord<double>(&tws_pair.second)};
    waveform::Trim(*trace, tws_pair.second);
    if (!proc.processor->Feed(trace)) {

      const auto &status{proc.processor->status()};
      const auto &status_value{proc.processor->status_value()};
      const auto &tw{tws_pair.second};
      SCDETECT_LOG_ERROR_TAGGED(
          proc.processor->id(),
          "%s: failed to feed data (tw.start=%s, "
          "tw.end=%s) to processor. Reason: status=%d, "
          "status_value=%f",
          trace->streamID().c_str(), tw.startTime().iso().c_str(),
          tw.endTime().iso().c_str(), utils::as_integer(status), status_value);

      return false;
    }

    if (detect::WaveformProcessor::Status::kWaitingForData ==
        proc.processor->status()) {
      return false;
    }
  }

  return true;
}

void Detector::PrepareResult(const Linker::Result &linker_res,
                             Detector::Result &res) const {
  const auto &ref_result{linker_res.results.at(linker_res.ref_proc_id)};
  if (!ref_result.match_result) {
    throw ProcessingError{"Failed to prepare result. Reason: missing reference "
                          "processor match result"};
  }

  const auto ref_match_result{ref_result.match_result};
  const auto &ref_starttime{ref_match_result->time_window.startTime()};
  const auto ref_pick_offset{
      static_cast<double>(ref_result.arrival.pick.time - ref_starttime)};

  std::unordered_set<std::string> used_chas;
  std::unordered_set<std::string> used_stas;
  // pick offsets (detected arrivals)
  std::vector<double> pick_offsets;

  // TODO(damb): Compute the uncertainty of the arrival time
  // w.r.t.:
  // - the alignment offset of the original waveforms (obtained from
  // result.time_window)
  //
  // Compute the uncertainty of the origin time under consideration of the:
  // - the arrival offsets w.r.t. the reference arrival
  const auto &ref_stream_id{ref_result.arrival.pick.waveform_id};
  const auto pot_offsets{linker_res.pot.GetOffsets(ref_stream_id)};
  Detector::Result::TemplateResults template_results;
  for (const auto &res_pair : linker_res.results) {
    const auto &proc_id{res_pair.first};
    const auto &templ_res{res_pair.second};
    const auto &proc{processors_.at(proc_id)};

    if (templ_res.match_result) {
      // compute alignment correction using the POT offset (required, since
      // traces to be cross-correlated cannot be guaranteed to be aligned to
      // sub-sampling interval accuracy)
      const auto &stream_id{templ_res.arrival.pick.waveform_id};
      const auto n{std::find_if(std::begin(pot_offsets), std::end(pot_offsets),
                                [&stream_id](const PickOffsetNode &n) {
                                  return n.stream_id == stream_id;
                                })};
      if (n == pot_offsets.end()) {
        throw ProcessingError{
            "Failed to prepare result. Reason: failed to lookup "
            "offset from POT"};
      }

      const auto match_result{templ_res.match_result};
      const auto &starttime{match_result->time_window.startTime()};
      const auto &alignment_correction{
          ref_starttime + Core::TimeSpan{n->pick_offset} - starttime};

      pick_offsets.push_back(
          static_cast<double>(templ_res.arrival.pick.time - starttime) -
          alignment_correction - ref_pick_offset);

      template_results.emplace(templ_res.arrival.pick.waveform_id,
                               Detector::Result::TemplateResult{
                                   templ_res.arrival, proc.sensor_location});
      used_chas.emplace(templ_res.arrival.pick.waveform_id);
      used_stas.emplace(proc.sensor_location.station_id);
    }
  }

  // compute origin time
  const auto &pick_offset_correction{
      utils::CMA(pick_offsets.data(), pick_offsets.size())};
  const auto &ref_origin_pick_offset{ref_result.arrival.pick.offset};
  res.origin_time = ref_starttime + Core::TimeSpan{ref_pick_offset} -
                    ref_origin_pick_offset +
                    Core::TimeSpan{pick_offset_correction};
  res.fit = linker_res.fit;
  // template results i.e. theoretical arrivals including some meta data
  res.template_results = template_results;
  // number of channels used
  res.num_channels_used = used_chas.size();
  // number of stations used
  res.num_stations_used = used_stas.size();
  // number of channels/stations associated
  std::unordered_set<std::string> assoc_stas;
  for (const auto &proc_pair : processors_) {
    assoc_stas.emplace(proc_pair.second.sensor_location.station_id);
  }
  res.num_channels_associated = linker_.GetAssociatedChannelCount();
  res.num_stations_associated = assoc_stas.size();
}

void Detector::ResetProcessing() {
  current_result_ = boost::none;

  ResetProcessors();
  // enable processors
  for (auto &proc_pair : processors_) {
    proc_pair.second.processor->enable();
  }

  ResetTrigger();
}

void Detector::ResetTrigger() {
  trigger_proc_id_ = boost::none;
  trigger_end_ = boost::none;
}

void Detector::ResetProcessors() {
  std::for_each(
      std::begin(processors_), std::end(processors_),
      [](ProcessorStates::value_type &p) { p.second.processor->Reset(); });
}

void Detector::EmitResult(const Detector::Result &res) {
  if (result_callback_) {
    result_callback_.value()(res);
  }
}

void Detector::StoreTemplateResult(
    const detect::WaveformProcessor *proc, const Record *rec,
    const detect::WaveformProcessor::ResultCPtr &res) {
  if (!proc || !rec || !res) {
    return;
  }

  auto &p{processors_.at(proc->id())};
  const auto &status{p.processor->status()};
  const auto &status_value{p.processor->status_value()};
  if (detect::WaveformProcessor::Status::kFinished == status &&
      100 == status_value) {

#ifdef SCDETECT_DEBUG
    const auto &match_result{
        boost::dynamic_pointer_cast<const Template::MatchResult>(res)};
    const auto &tw{proc->processed()};
    SCDETECT_LOG_DEBUG_PROCESSOR(
        proc, "[%s] (%-27s - %-27s): fit=%9f, lag=%10f",
        rec->streamID().c_str(), tw.startTime().iso().c_str(),
        tw.endTime().iso().c_str(), match_result->coefficient,
        match_result->lag);
#endif

    linker_.Feed(proc, res);
  } else {
    SCDETECT_LOG_WARNING_PROCESSOR(
        this,
        "Failed to match template (proc_id=%s). Reason : status=%d, "
        "status_value=%f",
        p.processor->id().c_str(), utils::as_integer(status), status_value);
  }
}

void Detector::StoreLinkerResult(const Linker::Result &res) {
  result_queue_.emplace_back(res);
}

} // namespace detector
} // namespace detect
} // namespace Seiscomp
