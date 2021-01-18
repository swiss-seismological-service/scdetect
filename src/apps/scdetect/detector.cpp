#include "detector.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <seiscomp/client/inventory.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/sensorlocation.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include "eventstore.h"
#include "log.h"
#include "settings.h"
#include "template.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {

Detector::Detector(const std::string &id) : Processor{id} {}

Detector::Detection::Detection(
    const double fit, const Core::Time &time, const double magnitude,
    const double lat, const double lon, const double depth,
    const Detector::Detection::SensorLocations &sensor_locations,
    const size_t num_stations_associated, const size_t num_stations_used,
    const size_t num_channels_associated, const size_t num_channels_used,
    const Detector::Detection::TemplateResults &template_results)
    : fit(fit), time(time), magnitude(magnitude), latitude(lat), longitude(lon),
      depth(depth), sensor_locations(sensor_locations),
      num_stations_associated(num_stations_associated),
      num_stations_used(num_stations_used),
      num_channels_associated(num_channels_associated),
      num_channels_used(num_channels_used), template_results(template_results) {
}

DetectorBuilder Detector::Create(const std::string &detector_id,
                                 const std::string &origin_id) {
  return DetectorBuilder(detector_id, origin_id);
}

void Detector::set_filter(Filter *filter) {
  if (!filter)
    return;

  for (const auto &config : stream_configs_)
    config.second.processor->set_filter(filter->clone());

  delete filter;
}

void Detector::set_gap_tolerance(const Core::TimeSpan &duration) {
  config_.gap_tolerance = static_cast<double>(duration);
}

const Core::TimeSpan Detector::gap_tolerance() const {
  return Core::TimeSpan{config_.gap_tolerance};
}

void Detector::set_gap_interpolation(bool e) { config_.gap_interpolation = e; }

bool Detector::gap_interpolation() const { return config_.gap_interpolation; }

bool Detector::Feed(const Record *record) {
  if (record->sampleCount() == 0)
    return false;

  auto it{stream_configs_.find(record->streamID())};
  if (it == stream_configs_.end())
    return false;

  return Store(it->second.stream_state, record);
}

void Detector::Reset() {
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Resetting detector ...");

  // reset template (child) related facilities
  for (auto &stream_config_pair : stream_configs_) {
    stream_config_pair.second.stream_state = Processor::StreamState{};
    stream_config_pair.second.stream_buffer->clear();
  }
  ResetProcessing();

  Processor::Reset();
}

std::string Detector::DebugString() const {

  bool first_result{true};
  std::ostringstream oss;
  oss << "{\"detectorId\": \"" << id() << "\", \"ccDebugInfo\": [" << std::endl;
  for (const auto &debug_result_pair : debug_cc_results_) {
    if (first_result) {
      first_result = false;
    } else {
      oss << ",";
    }

    oss << "{\"streamId\": \"" << debug_result_pair.first << "\", "
        << *debug_result_pair.second << "}" << std::endl;
  }
  oss << "]}";
  return oss.str();
}

bool Detector::WithPicks() const { return config_.create_picks; }

void Detector::Process(StreamState &stream_state, RecordCPtr record,
                       const DoubleArray &filtered_data) {

  const auto &config{config_};
  const auto &trigger_end{processing_state_.trigger_end};
  auto WithTrigger = [config]() { return config.trigger_duration > 0; };
  auto Triggered = [config, trigger_end]() {
    return config.trigger_duration > 0 && trigger_end;
  };
  auto NotTriggered = [config, trigger_end]() {
    return config.trigger_duration > 0 && !trigger_end;
  };

  // Returns the maximum buffered time window of data for stream buffers
  // identified by `stream_ids`. If `strict` is `true`, and the buffered
  // endtime is older than the `Detector`'s configured maximum allowed data
  // latency, an empty time window (i.e. duration is equal to zero) is
  // returned. Else the corresponding buffer is skipped when determining the
  // time window.
  auto FindBufferedTimeWindow =
      [this, config](const std::vector<WaveformStreamID> &stream_ids,
                     bool strict) -> Core::TimeWindow {
    // find max buffered endtime
    Core::Time max_endtime{};
    for (const auto &stream_id : stream_ids) {
      const auto it{stream_configs_.find(stream_id)};
      if (it == stream_configs_.end())
        continue;

      const auto &endtime{it->second.stream_buffer->timeWindow().endTime()};
      if (endtime && endtime > max_endtime)
        max_endtime = endtime;
    }

    Core::TimeWindow tw{};
    for (const auto &stream_id : stream_ids) {
      const auto it{stream_configs_.find(stream_id)};
      if (it == stream_configs_.end())
        continue;

      const auto &buffered_tw{it->second.stream_buffer->timeWindow()};

      // Ignore data with too high latency
      if (config.maximum_latency > 0 &&
          static_cast<double>(max_endtime - buffered_tw.endTime()) >
              config.maximum_latency) {

        if (strict) {
          tw.setLength(0);
          return tw;
        }

        continue;
      }

      if (!tw)
        tw = buffered_tw;
      else
        tw = tw.overlap(buffered_tw);
    }

    return tw;
  };

  // prepare data to be processed
  std::vector<WaveformStreamID> stream_ids;
  bool strict{true};
  if (WithTrigger() && processing_state_.trigger_end) {
    // Once triggered, only take those streams into consideration which
    // are already part of the processing procedure.
    auto if_enabled =
        [](const decltype(
            processing_state_.processor_states)::value_type &pair) {
          return pair.second.processor->enabled();
        };

    stream_ids =
        utils::filter_keys(processing_state_.processor_states, if_enabled);
  } else {
    auto if_enabled = [](const decltype(stream_configs_)::value_type &pair) {
      return pair.second.processor->enabled();
    };

    stream_ids = utils::filter_keys(stream_configs_, if_enabled);
    strict = false;
  }

  auto tw{FindBufferedTimeWindow(stream_ids, strict)};
  if (!tw)
    return;

  if (processed()) {
    if (tw.endTime() <= processed().endTime())
      return;
    // TODO(damb): Define margin?
    tw.setStartTime(processed().endTime());
  }

  // process templates i.e. compute cross-correlations
  bool waiting_for_data{false}, feeding_data_failed{false};
  for (const auto &stream_config_pair : stream_configs_) {
    // TODO(damb): Check if the stream was actually used. This info might be
    // saved while computing the time window to be processed.
    auto const &buffered_tw{
        stream_config_pair.second.stream_buffer->timeWindow()};
    if (!buffered_tw.contains(tw)) {
      stream_config_pair.second.processor->enable();
      continue;
    }

    // initialize processing related stream states
    if (!WithTrigger() || !processing_state_.trigger_end) {
      processing_state_.processor_states[stream_config_pair.first] =
          ProcessingState::ProcessorState{};
    }

    auto trace{
        stream_config_pair.second.stream_buffer->contiguousRecord<double>(&tw)};

    if (!stream_config_pair.second.processor->Feed(trace)) {
      feeding_data_failed = true;
      // feeding/storing data failed (not processing related)
      const auto status{stream_config_pair.second.processor->status()};
      const auto status_value{
          stream_config_pair.second.processor->status_value()};

      SCDETECT_LOG_ERROR_PROCESSOR(
          this,
          "%s: Failed to feed data to processor. Reason: status=%d, "
          "status_value=%f",
          stream_config_pair.first.c_str(), utils::as_integer(status),
          status_value);
      break;
    }

    if (Status::kWaitingForData ==
        stream_config_pair.second.processor->status()) {
      waiting_for_data = true;
    }
  }

  // XXX(damb): Regarding `waiting_for_data`, this is a workaround. Actually, it
  // would be better to make use of the Templates' internal buffers (similiar to
  // Seiscomp::Processing::TimeWindowProcessor). On the other hand, this way it
  // is easier to track that exactly the same, overlapping time window was
  // processed.
  if (feeding_data_failed || waiting_for_data) {
    if (processing_state_.trigger_end) {
      ResetProcessors();
    } else {
      ResetProcessing();
    }
    return;
  }

  // compute the fit (i.e. currently mean coefficient)
  double fit{0};
  size_t xcorr_failed{0};
  Core::TimeWindow min_correlated;
  for (const auto &processor_state_pair : processing_state_.processor_states) {

    const auto &processor{
        stream_configs_.at(processor_state_pair.first).processor};

    if (!processor_state_pair.second.result) {
      ++xcorr_failed;

      std::string msg{processor_state_pair.first +
                      ": Failed to match template. Reason: "};
      if (processor->enabled()) {
        const auto status{processor->status()};
        const auto status_value{processor->status_value()};
        msg += "status=" + std::to_string(utils::as_integer(status)) +
               ", status_value=" + std::to_string(status_value);
      } else {
        msg += "unknown (processor disabled)";
      }
      SCDETECT_LOG_WARNING_PROCESSOR(this, "%s", msg.c_str());

      continue;
    }

    fit += processor_state_pair.second.result->coefficient;

    if (!min_correlated) {
      min_correlated = processor->processed();
    } else if (min_correlated.endTime() > processor->processed().endTime()) {
      min_correlated.setEndTime(processor->processed().endTime());
    }

    if (debug_mode() && record->streamID() == processor_state_pair.first) {
      debug_cc_results_.emplace(record->streamID(),
                                processor_state_pair.second.result);
    }
  }

  if (xcorr_failed) {
    set_status(Status::kError, xcorr_failed);
    return;
  }

  fit /= processing_state_.processor_states.size();

  merge_processed(min_correlated);

  auto CreateStatsMsg = [config, &tw](const double &fit) {
    return std::string{
        "[" + tw.startTime().iso() + " - " + tw.endTime().iso() +
        "] fit=" + std::to_string(fit) +
        ", trigger_on=" + std::to_string(config.trigger_on) +
        ", trigger_off=" + std::to_string(config.trigger_off) +
        ", trigger_duration=" + std::to_string(config.trigger_duration)};
  };

  bool reset_processing{false};
  if (fit >= config_.trigger_on) {
    // initialize trigger
    if (NotTriggered()) {
      SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector triggered: %s",
                                   CreateStatsMsg(fit).c_str());

      processing_state_.trigger_end =
          tw.endTime() + Core::TimeSpan{config_.trigger_duration};
    } else if (!WithTrigger()) {
      SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result: %s",
                                   CreateStatsMsg(fit).c_str());
    } else {
      SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result (triggered): %s",
                                   CreateStatsMsg(fit).c_str());
    }

    if ((!WithTrigger() || processing_state_.trigger_end) &&
        (!std::isfinite(processing_state_.result.fit) ||
         fit > processing_state_.result.fit)) {
      // TODO(damb): Fit magnitude
      const auto processor_state_pair{
          processing_state_.processor_states.find(record->streamID())};
      if (processor_state_pair == processing_state_.processor_states.end()) {
        set_status(Status::kInvalidStream, 0);
        return;
      }

      auto origin_time{
          tw.startTime() +
          Core::TimeSpan{processor_state_pair->second.result->lag}};

      processing_state_.result.origin_time = origin_time;
      processing_state_.result.fit = fit;
      processing_state_.result.magnitude = magnitude_->magnitude().value();

      if (WithPicks()) {
        processing_state_.result.time_window = tw;
        for (const auto &processor_state_pair :
             processing_state_.processor_states) {
          processing_state_.result.lags.emplace(
              processor_state_pair.first,
              processor_state_pair.second.result->lag);
        }
      }
    }

    ResetProcessors();
  } else if (Triggered() && fit < config_.trigger_on &&
             fit >= config_.trigger_off) {

    SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result: %s",
                                 CreateStatsMsg(fit).c_str());
    ResetProcessors();
  } else {
    SCDETECT_LOG_DEBUG_PROCESSOR(this, "Detector result: %s",
                                 CreateStatsMsg(fit).c_str());

    if (!WithTrigger() || !processing_state_.trigger_end) {
      reset_processing = true;
    }
  }

  if ((!WithTrigger() && processing_state_.result.fit >= config_.trigger_on) ||
      (Triggered() && processed().endTime() >= processing_state_.trigger_end) ||
      (Triggered() && fit < config_.trigger_off)) {

    SCDETECT_LOG_INFO_PROCESSOR(
        this, "Detection: %s",
        CreateStatsMsg(processing_state_.result.fit).c_str());

    auto CountStations =
        [this](const std::vector<WaveformStreamID> &stream_ids) {
          std::unordered_set<WaveformStreamID> used_stations{};

          for (const auto &stream_id : stream_ids) {
            auto it{stream_configs_.find(stream_id)};
            if (it == stream_configs_.end())
              continue;

            auto station{it->second.metadata.sensor_location->station()};
            used_stations.insert(station->publicID());
          }

          return used_stations.size();
        };

    const auto &processor_states = processing_state_.processor_states;
    auto LoadSensorLocations = [this, processor_states]() {
      Detection::SensorLocations sensor_locations{};
      for (const auto &processor_state_pair : processor_states) {
        auto it{stream_configs_.find(processor_state_pair.first)};
        if (it == stream_configs_.end())
          continue;

        sensor_locations.push_back(it->second.metadata.sensor_location);
      }
      return sensor_locations;
    };

    std::vector<WaveformStreamID> stream_ids_used{};
    boost::copy(processing_state_.processor_states | boost::adaptors::map_keys,
                std::back_inserter(stream_ids_used));
    auto num_stations_used{CountStations(stream_ids_used)};
    std::vector<WaveformStreamID> stream_ids{};
    boost::copy(stream_configs_ | boost::adaptors::map_keys,
                std::back_inserter(stream_ids));
    auto num_stations_associated{stream_ids.size()};
    auto num_channels_used{processing_state_.processor_states.size()};
    auto num_channels_associated{stream_configs_.size()};
    auto sensor_locations{LoadSensorLocations()};

    Detection::TemplateResults template_results;
    for (const auto &processor_state_pair :
         processing_state_.processor_states) {

      Core::Time time_lag{};
      if (WithPicks()) {
        // compute pick times
        auto lag_pair{
            processing_state_.result.lags.find(processor_state_pair.first)};
        if (lag_pair == processing_state_.result.lags.end()) {
          continue;
        }
        time_lag = processing_state_.result.time_window.startTime() +
                   Core::TimeSpan{lag_pair->second};
      }
      Detection::TemplateResult tr{
          time_lag, processor_state_pair.second.result->metadata};

      template_results.emplace(processor_state_pair.first, tr);
    }

    ResultPtr detection{utils::make_smart<Detection>(
        processing_state_.result.fit,
        processing_state_.result.origin_time +
            Core::TimeSpan(config_.time_correction),
        processing_state_.result.magnitude, origin_->latitude().value(),
        origin_->longitude().value(), origin_->depth().value(),
        sensor_locations, num_stations_associated, num_stations_used,
        num_channels_associated, num_channels_used, template_results)};

    EmitResult(record, detection);
    reset_processing = true;

    // TODO(damb): Implement and set trigger_dead_time
  }

  if (reset_processing) {
    ResetProcessing();
  }
}

bool Detector::HandleGap(StreamState &stream_state, RecordCPtr record,
                         DoubleArrayPtr data) {

  if (stream_state.last_record) {
    if (record == stream_state.last_record)
      return false;

    const auto min_thres{2 * 1.0 / record->samplingFrequency()};
    if (min_thres > config_.gap_threshold) {
      SCDETECT_LOG_WARNING_PROCESSOR(
          this,
          "Gap threshold smaller than twice the sampling interval: %fs > %fs. "
          "Resetting gap threshold.",
          min_thres, config_.gap_threshold);

      config_.gap_threshold = min_thres;
    }

    Core::TimeSpan gap{record->startTime() -
                       stream_state.data_time_window.endTime() -
                       /* one usec*/ Core::TimeSpan(0, 1)};
    double gap_seconds = static_cast<double>(gap);

    if (gap > Core::TimeSpan{config_.gap_threshold}) {
      size_t gap_samples = static_cast<size_t>(
          ceil(stream_state.sampling_frequency * gap_seconds));
      if (FillGap(stream_state, record, gap, (*data)[0], gap_samples)) {
        SCDETECT_LOG_DEBUG_PROCESSOR(
            this, "%s: detected gap (%.6f secs, %lu samples) (handled)",
            record->streamID().c_str(), gap_seconds, gap_samples);
      } else {
        SCDETECT_LOG_DEBUG_PROCESSOR(
            this,
            "%s: detected gap (%.6f secs, %lu samples) (NOT "
            "handled): status=%d",
            record->streamID().c_str(), gap_seconds, gap_samples,
            // TODO(damb): Verify if this is the correct status
            // to be displayed
            static_cast<int>(status()));
        if (status() > Processor::Status::kInProgress)
          return false;
      }
    } else if (gap_seconds < 0) {
      // handle record from the past
      size_t gap_samples = static_cast<size_t>(
          ceil(-1 * stream_state.sampling_frequency * gap_seconds));
      if (gap_samples > 1)
        return false;
    }
    stream_state.data_time_window.setEndTime(record->endTime());
  }
  return true;
}

void Detector::Fill(StreamState &stream_state, RecordCPtr record, size_t n,
                    double *samples) {
  // XXX(damb): The detector does not filter the data. Data is buffered, only.

  stream_state.received_samples += n;

  // buffer record
  auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};
  if (!buffer->feed(record.get())) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        this, "%s: Error while buffering data: start=%s, end=%s, samples=%d",
        record->streamID().c_str(), record->startTime().iso().c_str(),
        record->endTime().iso().c_str(), record->sampleCount());
  }
}

bool Detector::EnoughDataReceived(const StreamState &stream_state) const {
  for (const auto &stream_config_pair : stream_configs_) {
    const auto &state{stream_config_pair.second.stream_state};
    if (state.received_samples <= state.needed_samples) {
      return false;
    }
  }
  return true;
}

void Detector::StoreTemplateResult(ProcessorCPtr processor, RecordCPtr record,
                                   ResultCPtr result) {
  if (!record || !result)
    return;

  auto it{processing_state_.processor_states.find(record->streamID())};
  if (it == processing_state_.processor_states.end())
    return;

  it->second.processor = processor;
  it->second.trace = record;
  it->second.result =
      boost::dynamic_pointer_cast<const Template::MatchResult>(result);
}

void Detector::ResetProcessing() {
  processing_state_ = ProcessingState{};

  ResetProcessors();
}

void Detector::ResetProcessors() {
  for (auto &stream_config_pair : stream_configs_) {
    stream_config_pair.second.processor->Reset();
  }
}

bool Detector::FillGap(StreamState &stream_state, RecordCPtr record,
                       const Core::TimeSpan &duration, double next_sample,
                       size_t missing_samples) {
  if (duration <= Core::TimeSpan{config_.gap_tolerance}) {
    if (config_.gap_interpolation) {

      auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};

      auto filled{utils::make_smart<GenericRecord>(
          record->networkCode(), record->stationCode(), record->locationCode(),
          record->channelCode(), buffer->timeWindow().endTime(),
          record->samplingFrequency())};

      auto data_ptr{utils::make_unique<DoubleArray>(missing_samples)};
      double delta{next_sample - stream_state.last_sample};
      double step{1. / static_cast<double>(missing_samples + 1)};
      double di = step;
      for (size_t i = 0; i < missing_samples; ++i, di += step) {
        const double value{stream_state.last_sample + di * delta};
        data_ptr->set(i, value);
      }

      filled->setData(missing_samples, data_ptr->typedData(), Array::DOUBLE);
      Fill(stream_state, /*record=*/filled, missing_samples,
           data_ptr->typedData());

      return true;
    }
  }

  return false;
}

/* -------------------------------------------------------------------------- */
// XXX(damb): Using `new` to access a non-public ctor; see also
// https://abseil.io/tips/134
DetectorBuilder::DetectorBuilder(const std::string &detector_id,
                                 const std::string &origin_id)
    : origin_id_{origin_id}, detector_{new Detector{detector_id}} {

  if (!set_origin(origin_id_)) {
    throw builder::BaseException{std::string{"Error while assigning origin: "} +
                                 origin_id};
  }
}

DetectorBuilder &DetectorBuilder::set_config(const DetectorConfig &config) {
  detector_->config_ = config;

  detector_->enabled_ = config.enabled;

  return *this;
}

DetectorBuilder &DetectorBuilder::set_eventparameters() {

  DataModel::EventParametersPtr event_parameters{
      detector_->origin_->eventParameters()};
  // find the origin's associated event
  bool found{false};
  for (size_t i = 0; i < event_parameters->eventCount(); ++i) {
    DataModel::EventPtr event = event_parameters->event(i);
    for (size_t j = 0; j < event->originReferenceCount(); ++j) {
      DataModel::OriginReferencePtr origin_ref = event->originReference(j);
      if (origin_ref->originID() == origin_id_) {
        detector_->event_ = event;
        found = true;
        break;
      }
    }
    if (found)
      break;
  }

  if (!found) {
    auto msg{std::string{"No event associated with origin: "} + origin_id_};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  detector_->magnitude_ = EventStore::Instance().Get<DataModel::Magnitude>(
      detector_->event_->preferredMagnitudeID());

  if (!detector_->magnitude_) {
    auto msg{std::string{"No magnitude associated with event: "} +
             detector_->event_->publicID() + std::string{" (origin="} +
             origin_id_ + std::string{")"}};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  return *this;
}

DetectorBuilder &DetectorBuilder::set_stream(
    const std::string &stream_id, const StreamConfig &stream_config,
    WaveformHandlerIfacePtr waveform_handler, double gap_tolerance,
    const boost::filesystem::path &path_debug_info)

{
  const auto &template_stream_id{stream_config.template_config.wf_stream_id};
  utils::WaveformStreamID template_wf_stream_id{template_stream_id};

  // TODO(damb): Rather go for a logging adapter approach.
  std::string log_prefix{stream_id + std::string{" ("} + template_stream_id +
                         std::string{"): "}};

  // configure pick from arrival
  DataModel::PickPtr pick;
  DataModel::WaveformStreamID pick_waveform_id;
  double arrival_weight;
  for (size_t i = 0; i < detector_->origin_->arrivalCount(); ++i) {
    DataModel::ArrivalPtr arrival{detector_->origin_->arrival(i)};

    if (arrival->phase().code() != stream_config.template_config.phase) {
      continue;
    }

    pick = EventStore::Instance().Get<DataModel::Pick>(arrival->pickID());
    if (!IsValidArrival(arrival, pick)) {
      continue;
    }

    // compare sensor locations
    try {
      pick->time().value();
    } catch (...) {
      continue;
    }
    auto template_wf_sensor_location{
        Client::Inventory::Instance()->getSensorLocation(
            template_wf_stream_id.net_code(), template_wf_stream_id.sta_code(),
            template_wf_stream_id.loc_code(), pick->time().value())};
    if (!template_wf_sensor_location) {
      auto msg{log_prefix +
               std::string{
                   "Sensor location not found in inventory for time: time="} +
               pick->time().value().iso()};

      SCDETECT_LOG_WARNING("%s", msg.c_str());
      throw builder::NoSensorLocation{msg};
    }
    pick_waveform_id = pick->waveformID();
    auto pick_wf_sensor_location{
        Client::Inventory::Instance()->getSensorLocation(
            pick_waveform_id.networkCode(), pick_waveform_id.stationCode(),
            pick_waveform_id.locationCode(), pick->time().value())};
    if (!pick_wf_sensor_location ||
        *template_wf_sensor_location != *pick_wf_sensor_location) {
      continue;
    }

    arrival_weight = arrival->weight();
    break;
  }

  if (!pick) {
    auto msg{log_prefix + std::string{"Failed to load pick: origin="} +
             origin_id_ + std::string{", phase="} +
             stream_config.template_config.phase};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  std::ostringstream oss;
  oss << utils::WaveformStreamID{pick_waveform_id};
  SCDETECT_LOG_DEBUG(
      "%sUsing arrival pick: origin=%s, time=%s, phase=%s, stream=%s",
      log_prefix.c_str(), origin_id_.c_str(),
      pick->time().value().iso().c_str(),
      stream_config.template_config.phase.c_str(), oss.str().c_str());

  auto wf_start{pick->time().value() +
                Core::TimeSpan{stream_config.template_config.wf_start}};
  auto wf_end{pick->time().value() +
              Core::TimeSpan{stream_config.template_config.wf_end}};

  // load stream metadata from inventory
  auto stream{Client::Inventory::Instance()->getStream(
      template_wf_stream_id.net_code(), template_wf_stream_id.sta_code(),
      template_wf_stream_id.loc_code(), template_wf_stream_id.cha_code(),
      wf_start)};

  if (!stream) {
    auto msg{log_prefix +
             std::string{"Stream not found in inventory for epoch: start="} +
             wf_start.iso() + std::string{", end="} + wf_end.iso()};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::NoStream{msg};
  }

  SCDETECT_LOG_DEBUG("%sLoaded stream from inventory for epoch: start=%s, "
                     "end=%s",
                     log_prefix.c_str(), wf_start.iso().c_str(),
                     wf_end.iso().c_str());

  // set template related filter (used for template waveform processing)
  WaveformHandlerIface::ProcessingConfig template_wf_config;
  template_wf_config.filter_string = stream_config.template_config.filter;

  detector_->stream_configs_[stream_id] = Detector::StreamConfig{};

  // create template related filter (used during real-time stream
  // processing)
  std::unique_ptr<Processor::Filter> rt_template_filter{nullptr};
  if (!stream_config.filter.empty()) {
    std::string err;
    rt_template_filter.reset(
        Processor::Filter::Create(stream_config.filter, &err));

    if (!rt_template_filter) {
      auto msg{log_prefix + std::string{"Compiling filter ("} +
               stream_config.filter + std::string{") failed: "} + err};

      SCDETECT_LOG_WARNING("%s", msg.c_str());
      throw builder::BaseException{msg};
    }
  }

  SCDETECT_LOG_DEBUG("Creating template processor (id=%s) ... ",
                     stream_config.template_id.c_str());

  detector_->stream_configs_[stream_id].processor =
      Template::Create(stream_config.template_id)
          .set_phase(stream_config.template_config.phase)
          .set_arrival_weight(arrival_weight)
          .set_pick(pick)
          .set_stream_config(*stream)
          .set_filter(rt_template_filter.release(), stream_config.init_time)
          .set_sensitivity_correction(stream_config.sensitivity_correction)
          .set_publish_callback(
              std::bind(&Detector::StoreTemplateResult, detector_.get(),
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3))
          .set_waveform(waveform_handler, template_stream_id, wf_start, wf_end,
                        template_wf_config)
          .set_debug_info_dir(path_debug_info)
          .build();

  const auto &template_init_time{
      detector_->stream_configs_[stream_id].processor->init_time()};
  if (template_init_time > detector_->init_time()) {
    detector_->init_time_ = template_init_time;
  }

  detector_->stream_configs_[stream_id].stream_buffer =
      utils::make_unique<RingBuffer>(
          template_init_time * settings::kBufferMultiplicator, gap_tolerance);

  detector_->stream_configs_[stream_id].metadata.sensor_location =
      stream->sensorLocation();

  return *this;
}

DetectorBuilder &DetectorBuilder::set_publish_callback(
    Processor::PublishResultCallback callback) {
  detector_->set_result_callback(callback);
  return *this;
}

DetectorBuilder &
DetectorBuilder::set_debug_info_dir(const boost::filesystem::path &path) {
  detector_->set_debug_info_dir(path);
  return *this;
}

DetectorPtr DetectorBuilder::build() { return detector_; }

bool DetectorBuilder::IsValidArrival(const DataModel::ArrivalCPtr arrival,
                                     const DataModel::PickCPtr pick) {
  if (!pick) {
    SCDETECT_LOG_DEBUG("Failed loading pick: %s", arrival->pickID().c_str());
    return false;
  }
  // check if both pick and arrival are properly configured
  try {
    if (pick->evaluationMode() != DataModel::MANUAL &&
        (arrival->weight() == 0 || !arrival->timeUsed())) {
      return false;
    }
  } catch (Core::ValueException &e) {
    return false;
  }
  return true;
}

bool DetectorBuilder::set_origin(const std::string &origin_id) {
  if (!detector_->origin_) {

    DataModel::OriginPtr origin{
        EventStore::Instance().Get<DataModel::Origin>(origin_id)};
    if (!origin) {
      SCDETECT_LOG_WARNING("Origin %s not found.", origin_id.c_str());
      return false;
    }
    detector_->origin_ = origin;
  }
  return true;
}

} // namespace detect
} // namespace Seiscomp
