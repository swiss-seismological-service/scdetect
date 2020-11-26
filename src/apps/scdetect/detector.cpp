#include "detector.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
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
#include "settings.h"
#include "template.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {

Detector::Detector() {}

Detector::Detection::Detection(
    const double fit, const Core::Time &time, const double magnitude,
    const double lat, const double lon, const double depth,
    const Detector::Detection::SensorLocations &sensor_locations,
    const size_t num_stations_associated, const size_t num_stations_used,
    const size_t num_channels_associated, const size_t num_channels_used,
    const Detector::Detection::TemplateResultMetaData &template_metadata)
    : fit(fit), time(time), magnitude(magnitude), latitude(lat), longitude(lon),
      depth(depth), sensor_locations(sensor_locations),
      num_stations_associated(num_stations_associated),
      num_stations_used(num_stations_used),
      num_channels_associated(num_channels_associated),
      num_channels_used(num_channels_used),
      template_metadata(template_metadata) {}

DetectorBuilder Detector::Create(const std::string &origin_id) {
  return DetectorBuilder(origin_id);
}

void Detector::set_filter(Filter *filter) {
  if (!filter)
    return;

  for (const auto &config : stream_configs_)
    config.second.processor->set_filter(filter->clone());

  delete filter;
}

bool Detector::Feed(const Record *record) {
  if (record->sampleCount() == 0)
    return false;

  auto it{stream_configs_.find(record->streamID())};
  if (it == stream_configs_.end())
    return false;

  return Store(it->second.stream_state, record);
}

void Detector::Process(StreamState &stream_state, RecordCPtr record,
                       const DoubleArray &filtered_data) {

  auto WithTriggerDuration = [this]() { return config_.trigger_duration > 0; };

  // Returns the maximum buffered time window of data for stream buffers
  // identified by `stream_ids`. If `strict` is `true`, and the buffered
  // endtime is older than the `Detector`'s configured maximum allowed data
  // latency, an empty time window (i.e. duration is equal to zero) is
  // returned. Else the corresponding buffer is skipped when determining the
  // time window.
  auto FindBufferedTimeWindow =
      [this](const std::vector<std::string> &stream_ids,
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
      if (config_.maximum_latency > 0 &&
          static_cast<double>(max_endtime - buffered_tw.endTime()) >
              config_.maximum_latency) {

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
  std::vector<std::string> stream_ids{};
  bool strict{true};
  if (WithTriggerDuration() && processing_state_.trigger_end) {
    // Once triggered, only take those streams into consideration which
    // are already part of the processing procedure.
    boost::copy(processing_state_.stream_states | boost::adaptors::map_keys,
                std::back_inserter(stream_ids));

  } else {
    boost::copy(stream_configs_ | boost::adaptors::map_keys,
                std::back_inserter(stream_ids));
    strict = false;
  }
  auto tw{FindBufferedTimeWindow(stream_ids, strict)};

  if (!tw)
    return;

  if (processed_) {
    if (tw.endTime() <= processed_.endTime())
      return;
    // TODO(damb): Define margin?
    tw.setStartTime(processed_.endTime());
  }

  // process templates i.e. compute cross-correlations
  for (const auto &pair : stream_configs_) {
    // TODO(damb): Check if the stream was actually used. This info might be
    // saved while computing the time window to be processed.
    auto const &buffered_tw{pair.second.stream_buffer->timeWindow()};
    if (!buffered_tw.contains(tw)) {
      pair.second.processor->enable();
      continue;
    }

    // initialize processing related stream states
    if (!processing_state_.trigger_end)
      processing_state_.stream_states[pair.first] =
          ProcessingState::StreamState{};

    auto trace{pair.second.stream_buffer->contiguousRecord<double>(&tw)};

    if (!pair.second.processor->Feed(trace)) {
      const auto status{pair.second.processor->status()};
      const auto status_value{pair.second.processor->status_value()};

      SEISCOMP_ERROR("%s: Failed to feed data to processor. Reason: status=%d, "
                     "status_value=%f",
                     pair.first.c_str(), utils::as_integer(status),
                     status_value);

      // TODO(damb): Storing the data failed. What to do next?
    }
  }

  processed_ | tw;

  bool xcorr_failed{false};
  // compute the mean coefficient
  double mean_coefficient{0};
  for (const auto &pair : processing_state_.stream_states) {
    if (!pair.second.result) {
      xcorr_failed = true;

      const auto status{pair.second.processor->status()};
      const auto status_value{pair.second.processor->status_value()};
      SEISCOMP_WARNING(
          "%s: Failed to match template. Skipping. Reason: status=%d, "
          "status_value=%f",
          pair.first.c_str(), utils::as_integer(status), status_value);

      continue;
    }
    mean_coefficient += pair.second.result->coefficient;
  }

  if (xcorr_failed) {
    ResetProcessing();
    return;
  }

  mean_coefficient /= processing_state_.stream_states.size();

  if (mean_coefficient >= config_.trigger_on) {
    // initialize trigger
    if (WithTriggerDuration() && !processing_state_.trigger_end) {
      processing_state_.trigger_end =
          tw.endTime() + Core::TimeSpan(config_.trigger_duration);
    }

    if (!WithTriggerDuration() || processing_state_.trigger_end) {
      // TODO(damb): Fit magnitude
      const auto it{processing_state_.stream_states.find(record->streamID())};
      if (it == processing_state_.stream_states.end())
        return;

      auto origin_time{it->second.trace->startTime() +
                       Core::TimeSpan(it->second.result->lag)};

      processing_state_.result = {origin_time, mean_coefficient,
                                  magnitude_->magnitude().value()};
    }
  }

  if (!WithTriggerDuration() ||
      (processing_state_.trigger_end &&
       processed_.endTime() >= processing_state_.trigger_end) ||
      (processing_state_.trigger_end &&
       mean_coefficient < config_.trigger_off)) {

    auto CountStations = [this](const std::vector<std::string> &stream_ids) {
      std::unordered_set<std::string> used_stations{};

      for (const auto &stream_id : stream_ids) {
        auto it{stream_configs_.find(stream_id)};
        if (it == stream_configs_.end())
          continue;

        auto station{it->second.metadata.sensor_location->station()};
        used_stations.insert(station->publicID());
      }

      return used_stations.size();
    };

    auto LoadSensorLocations = [this]() {
      Detection::SensorLocations sensor_locations{};
      for (const auto &state : processing_state_.stream_states) {
        auto it{stream_configs_.find(state.first)};
        if (it == stream_configs_.end())
          continue;

        sensor_locations.push_back(it->second.metadata.sensor_location);
      }
      return sensor_locations;
    };

    std::vector<std::string> stream_ids_used{};
    boost::copy(processing_state_.stream_states | boost::adaptors::map_keys,
                std::back_inserter(stream_ids_used));
    auto num_stations_used{CountStations(stream_ids_used)};
    std::vector<std::string> stream_ids{};
    boost::copy(stream_configs_ | boost::adaptors::map_keys,
                std::back_inserter(stream_ids));
    auto num_stations_associated{stream_ids.size()};
    auto num_channels_used{processing_state_.stream_states.size()};
    auto num_channels_associated{stream_configs_.size()};
    auto sensor_locations{LoadSensorLocations()};

    Detection::TemplateResultMetaData metadata{};
    for (const auto &pair : processing_state_.stream_states)
      metadata.insert(std::make_pair(pair.first, pair.second.result->metadata));

    ResultPtr detection{new Detection{
        processing_state_.result.fit,
        processing_state_.result.origin_time +
            Core::TimeSpan(config_.time_correction),
        processing_state_.result.magnitude, origin_->latitude().value(),
        origin_->longitude().value(), origin_->depth().value(),
        sensor_locations, num_stations_associated, num_stations_used,
        num_channels_associated, num_channels_used, metadata}};

    EmitResult(record, detection);
    ResetProcessing();

    // TODO(damb): Implement and set trigger_dead_time
  }
} // namespace detect

void Detector::Fill(StreamState &stream_state, RecordCPtr record, size_t n,
                    double *samples) {
  // XXX(damb): The detector does not filter the data. Data is buffered, only.
  stream_state.received_samples += n;

  // buffer filled data
  GenericRecordPtr filled{new GenericRecord{
      record->networkCode(), record->stationCode(), record->locationCode(),
      record->channelCode(), record->startTime(), record->samplingFrequency()}};
  filled->setData(n, samples, Array::DOUBLE);

  try {
    stream_configs_.at(record->streamID()).stream_buffer->feed(filled.get());
  } catch (std::out_of_range &e) {
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

  auto it{processing_state_.stream_states.find(record->streamID())};
  if (it == processing_state_.stream_states.end())
    return;

  it->second.processor = processor;
  it->second.trace = record;
  it->second.result =
      boost::dynamic_pointer_cast<const Template::MatchResult>(result);
}

void Detector::ResetProcessing() { processing_state_ = ProcessingState{}; }

/* -------------------------------------------------------------------------  */
DetectorBuilder::DetectorBuilder(const std::string &origin_id)
    : origin_id_(origin_id), detector_(new Detector{}) {

  if (!set_origin(origin_id_)) {
    throw builder::BaseException{std::string{"Error while assigning origin: "} +
                                 origin_id};
  }
}

DetectorBuilder &DetectorBuilder::set_config(const DetectorConfig &config) {
  detector_->config_ = config;

  detector_->enabled_ = config.enabled;

  detector_->gap_interpolation_ = config.gap_interpolation;
  detector_->gap_tolerance_ = config.gap_tolerance;

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
    SEISCOMP_WARNING("No event associated with origin %s.", origin_id_.c_str());
    throw builder::BaseException{
        std::string{"No event associated with origin: "} + origin_id_};
  }

  detector_->magnitude_ = EventStore::Instance().Get<DataModel::Magnitude>(
      detector_->event_->preferredMagnitudeID());

  if (!detector_->magnitude_) {
    SEISCOMP_WARNING("No magnitude associated with event %s: origin=%s",
                     detector_->event_->publicID().c_str(), origin_id_.c_str());
    throw builder::BaseException{
        std::string{"No magnitude associated with event: "} +
        detector_->event_->publicID() + std::string{" (origin="} + origin_id_ +
        std::string{")"}};
  }

  return *this;
}

DetectorBuilder &
DetectorBuilder::set_stream(const std::string &stream_id,
                            const StreamConfig &stream_config,
                            WaveformHandlerIfacePtr waveform_handler) {

  const auto &template_stream_id{stream_config.template_config.wf_stream_id};

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

    pick_waveform_id = pick->waveformID();
    if (template_stream_id != static_cast<std::string>(pick_waveform_id)) {
      continue;
    }

    arrival_weight = arrival->weight();
    break;
  }

  if (!pick) {
    SEISCOMP_WARNING("%s (%s): Failed to load pick: origin=%s, phase=%s",
                     stream_id.c_str(), template_stream_id.c_str(),
                     detector_->origin_->publicID().c_str(),
                     stream_config.template_config.phase.c_str());

    throw builder::BaseException{
        stream_id + std::string{" ("} + template_stream_id +
        std::string{"): Failed to load pick: origin="} + origin_id_ +
        std::string{", phase="} + stream_config.template_config.phase};
  }

  try {
    pick->time().value();
  } catch (...) {
    SEISCOMP_WARNING(
        "%s (%s): Failed to load pick (invalid time): origin=%s, phase=%s",
        stream_id.c_str(), template_stream_id.c_str(),
        detector_->origin_->publicID().c_str(),
        stream_config.template_config.phase.c_str());

    throw builder::BaseException{
        stream_id + std::string{" ("} + template_stream_id +
        std::string{"): Failed to load pick (invalid time): origin="} +
        origin_id_ + std::string{", phase="} +
        stream_config.template_config.phase};
  }

  auto wf_start{pick->time().value() +
                Core::TimeSpan(stream_config.template_config.wf_start)};
  auto wf_end{pick->time().value() +
              Core::TimeSpan(stream_config.template_config.wf_end)};

  // load stream metadata from inventory
  auto stream{Client::Inventory::Instance()->getStream(
      pick_waveform_id.networkCode(), pick_waveform_id.stationCode(),
      pick_waveform_id.locationCode(), pick_waveform_id.channelCode(),
      wf_start)};

  if (!stream) {
    SEISCOMP_WARNING(
        "%s (%s): Stream not found in inventory for epoch: start=%s, "
        "end=%s",
        stream_id.c_str(), template_stream_id.c_str(), wf_start.iso().c_str(),
        wf_end.iso().c_str());

    throw builder::NoStream{
        stream_id + std::string{" ("} + template_stream_id +
        std::string{"): Stream not found in inventory for epoch: start="} +
        wf_start.iso() + std::string{", end="} + wf_end.iso()};
  }

  SEISCOMP_DEBUG("%s (%s): Loaded stream from inventory for epoch: start=%s, "
                 "end=%s",
                 stream_id.c_str(), template_stream_id.c_str(),
                 wf_start.iso().c_str(), wf_end.iso().c_str());

  WaveformHandlerIface::ProcessingConfig config;
  config.filter_string = stream_config.filter;

  detector_->stream_configs_[stream_id] = Detector::StreamConfig{};

  // create template related filter (used during real-time stream processing)
  std::unique_ptr<Processor::Filter> rt_template_filter;
  if (!stream_config.filter.empty()) {
    std::string err;
    rt_template_filter.reset(
        Processor::Filter::Create(stream_config.filter, &err));

    if (!rt_template_filter) {
      SEISCOMP_WARNING("%s (%s): Compiling filter (%s) failed: %s",
                       stream_id.c_str(), template_stream_id.c_str(),
                       stream_config.filter.c_str(), err.c_str());

      throw builder::BaseException{
          stream_id + std::string{" ("} + template_stream_id +
          std::string{"): Compiling filter ("} + stream_config.filter +
          std::string{") failed: "} + err};
    }
  }

  detector_->stream_configs_[stream_id].processor =
      Template::Create()
          .set_phase(stream_config.template_config.phase)
          .set_arrival_weight(arrival_weight)
          .set_pick(pick)
          .set_stream_config(*stream)
          .set_filter(rt_template_filter.release(), stream_config.init_time)
          .set_waveform(waveform_handler, template_stream_id, wf_start, wf_end,
                        config);

  const auto &template_init_time{
      detector_->stream_configs_[stream_id].processor->init_time()};
  if (template_init_time > detector_->init_time()) {
    detector_->init_time_ = template_init_time;
  }

  detector_->stream_configs_[stream_id].stream_buffer =
      utils::make_unique<RingBuffer>(
          detector_->stream_configs_[stream_id].processor->init_time() *
          settings::kBufferMultiplicator);

  detector_->stream_configs_[stream_id].metadata.sensor_location =
      stream->sensorLocation();

  return *this;
}

DetectorBuilder &DetectorBuilder::set_publish_callback(
    const Processor::PublishResultCallback &callback) {
  detector_->set_result_callback(callback);
  return *this;
}

DetectorBuilder::operator DetectorPtr() { return detector_; }

bool DetectorBuilder::IsValidArrival(const DataModel::ArrivalCPtr arrival,
                                     const DataModel::PickCPtr pick) {
  if (!pick) {
    SEISCOMP_DEBUG("Failed loading pick: %s", arrival->pickID().c_str());
    return false;
  }
  if (!utils::ValidatePhase(arrival->phase().code()))
    return false;
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
      SEISCOMP_WARNING("Origin %s not found.", origin_id.c_str());
      return false;
    }
    detector_->origin_ = origin;
  }
  return true;
}

} // namespace detect
} // namespace Seiscomp
