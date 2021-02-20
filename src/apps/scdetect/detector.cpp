#include "detector.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

#include <boost/none.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <seiscomp/client/inventory.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/exceptions.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/eventparameters.h>

#include "eventstore.h"
#include "log.h"
#include "processor.h"
#include "settings.h"
#include "template.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {

Detector::Detector(const std::string &id, const DataModel::OriginCPtr &origin)
    : Processor{id}, detector_{this, origin}, origin_{origin} {}

DetectorBuilder Detector::Create(const std::string &detector_id,
                                 const std::string &origin_id) {
  return DetectorBuilder(detector_id, origin_id);
}

void Detector::set_filter(Filter *filter) {
  // XXX(damb): `Detector` does not implement filter facilities.
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

  detector_.Reset();

  Processor::Reset();
}

void Detector::Terminate() {
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Terminating ...");

  detector_.Terminate();
  if (detection_) {
    auto detection{utils::make_smart<Detection>()};
    PrepareDetection(detection, *detection_);
    EmitResult(nullptr, detection);

    detection_ = boost::none;
  }
  Processor::Terminate();
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

void Detector::Process(StreamState &stream_state, const Record *record,
                       const DoubleArray &filtered_data) {
  try {
    detector_.Process(record->streamID());
  } catch (detector::Detector::ProcessingError &e) {
    SCDETECT_LOG_WARNING_PROCESSOR(this, "%s: %s", record->streamID().c_str(),
                                   e.what());
    detector_.Reset();
  } catch (std::exception &e) {
    SCDETECT_LOG_ERROR_PROCESSOR(this, "%s: Unhandled exception: %s",
                                 record->streamID().c_str(), e.what());

    set_status(Processor::Status::kError, 0);
  } catch (...) {
    SCDETECT_LOG_ERROR_PROCESSOR(this, "%s: Unknown exception.",
                                 record->streamID().c_str());

    set_status(Processor::Status::kError, 0);
  }

  if (!finished()) {
    merge_processed(detector_.processed());

    if (detection_) {
      auto detection{utils::make_smart<Detection>()};
      PrepareDetection(detection, *detection_);
      EmitResult(record, detection);

      detection_ = boost::none;
    }
  }
}

bool Detector::HandleGap(StreamState &stream_state, const Record *record,
                         DoubleArrayPtr &data) {

  if (record == stream_state.last_record)
    return false;

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

  return true;
}

void Detector::Fill(StreamState &stream_state, const Record *record,
                    DoubleArrayPtr &data) {
  // XXX(damb): The detector does not filter the data. Data is buffered, only.
  stream_state.received_samples += data->size();

  auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};
  // buffer record
  if (!buffer->feed(record)) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        this, "%s: Error while buffering data: start=%s, end=%s, samples=%d",
        record->streamID().c_str(), record->startTime().iso().c_str(),
        record->endTime().iso().c_str(), record->sampleCount());
  }
}

void Detector::InitStream(StreamState &stream_state, const Record *record) {
  Processor::InitStream(stream_state, record);

  const auto min_thres{2 * 1.0 / record->samplingFrequency()};
  if (min_thres > config_.gap_threshold) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        this,
        "Gap threshold smaller than twice the sampling interval: %fs > %fs. "
        "Resetting gap threshold.",
        min_thres, config_.gap_threshold);

    // TODO(damb): When implementing the feature/handle-changing-sampling
    // rates (see: https://github.com/damb/scdetect/issues/20) store remember
    // the configured gap threshold value and reset the current gap threshold
    // to the configured one, once the sampling interval decreases.
    config_.gap_threshold = min_thres;
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

void Detector::StoreDetection(const detector::Detector::Result &res) {
  detection_ = res;
}

void Detector::PrepareDetection(DetectionPtr &detection,
                                const detector::Detector::Result &res) {
  detection->fit = detection_.value().fit;
  detection->time = res.origin_time + Core::TimeSpan(config_.time_correction);
  detection->latitude = origin_->latitude().value();
  detection->longitude = origin_->longitude().value();
  detection->depth = origin_->depth().value();

  const auto &mag{res.magnitude};
  detection->magnitude = mag.value_or(magnitude_->magnitude().value());

  detection->num_channels_associated = res.num_channels_associated;
  detection->num_channels_used = res.num_channels_used;
  detection->num_stations_associated = res.num_stations_associated;
  detection->num_stations_used = res.num_stations_used;

  detection->template_results = res.template_results;
}

bool Detector::FillGap(StreamState &stream_state, const Record *record,
                       const Core::TimeSpan &duration, double next_sample,
                       size_t missing_samples) {
  if (duration <= Core::TimeSpan{config_.gap_tolerance}) {
    if (config_.gap_interpolation) {

      auto &buffer{stream_configs_.at(record->streamID()).stream_buffer};

      auto filled{utils::make_smart<GenericRecord>(
          record->networkCode(), record->stationCode(), record->locationCode(),
          record->channelCode(), buffer->timeWindow().endTime(),
          record->samplingFrequency())};

      auto data_ptr{utils::make_smart<DoubleArray>(missing_samples)};
      double delta{next_sample - stream_state.last_sample};
      double step{1. / static_cast<double>(missing_samples + 1)};
      double di = step;
      for (size_t i = 0; i < missing_samples; ++i, di += step) {
        const double value{stream_state.last_sample + di * delta};
        data_ptr->set(i, value);
      }

      filled->setData(missing_samples, data_ptr->typedData(), Array::DOUBLE);
      Fill(stream_state, /*record=*/filled.get(), data_ptr);

      return true;
    }
  }

  return false;
}

/* -------------------------------------------------------------------------- */
DetectorBuilder::DetectorBuilder(const std::string &detector_id,
                                 const std::string &origin_id)
    : origin_id_{origin_id} {

  DataModel::OriginCPtr origin{
      EventStore::Instance().Get<DataModel::Origin>(origin_id)};
  if (!origin) {
    SCDETECT_LOG_WARNING("Origin %s not found.", origin_id.c_str());
    throw builder::BaseException{std::string{"Error while assigning origin: "} +
                                 origin_id};
  }

  // XXX(damb): Using `new` to access a non-public ctor; see also
  // https://abseil.io/tips/134
  product_ = std::unique_ptr<Detector>(new Detector{detector_id, origin});
}

DetectorBuilder &DetectorBuilder::set_config(const DetectorConfig &config) {
  product_->config_ = config;

  product_->enabled_ = config.enabled;

  return *this;
}

DetectorBuilder &DetectorBuilder::set_eventparameters() {

  DataModel::EventParametersPtr event_parameters{
      product_->origin_->eventParameters()};
  // find the origin's associated event
  bool found{false};
  for (size_t i = 0; i < event_parameters->eventCount(); ++i) {
    DataModel::EventPtr event = event_parameters->event(i);
    for (size_t j = 0; j < event->originReferenceCount(); ++j) {
      DataModel::OriginReferencePtr origin_ref = event->originReference(j);
      if (origin_ref->originID() == origin_id_) {
        product_->event_ = event;
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

  product_->magnitude_ = EventStore::Instance().Get<DataModel::Magnitude>(
      product_->event_->preferredMagnitudeID());

  if (!product_->magnitude_) {
    auto msg{std::string{"No magnitude associated with event: "} +
             product_->event_->publicID() + std::string{" (origin="} +
             origin_id_ + std::string{")"}};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  return *this;
}

DetectorBuilder &
DetectorBuilder::set_stream(const std::string &stream_id,
                            const StreamConfig &stream_config,
                            WaveformHandlerIfacePtr waveform_handler,
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
  DataModel::ArrivalPtr arrival;
  for (size_t i = 0; i < product_->origin_->arrivalCount(); ++i) {
    arrival = product_->origin_->arrival(i);

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

    break;
  }

  if (!pick) {
    arrival.reset();
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

  SCDETECT_LOG_DEBUG("Creating template processor (id=%s) ... ",
                     stream_config.template_id.c_str());

  product_->stream_configs_[stream_id] = Detector::StreamConfig{};

  // set template related filter (used for template waveform processing)
  WaveformHandlerIface::ProcessingConfig template_wf_config;
  template_wf_config.filter_string = stream_config.template_config.filter;

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

  Core::Time start, end;
  // template processor
  auto template_proc{
      Template::Create(stream_config.template_id, product_.get())
          .set_stream_config(*stream)
          .set_filter(rt_template_filter.release(), stream_config.init_time)
          .set_sensitivity_correction(stream_config.sensitivity_correction)
          .set_waveform(waveform_handler, template_stream_id, wf_start, wf_end,
                        template_wf_config, start, end)
          .set_debug_info_dir(path_debug_info)
          .Build()};

  TemplateProcessorConfig c{
      std::move(template_proc),
      {stream->sensorLocation(), pick, arrival, pick->time().value() - start}};

  processor_configs_.emplace(stream_id, std::move(c));

  arrival_picks_.push_back(detector::POT::ArrivalPick{arrival, pick});

  return *this;
}

DetectorBuilder &
DetectorBuilder::set_debug_info_dir(const boost::filesystem::path &path) {
  product_->set_debug_info_dir(path);
  return *this;
}

void DetectorBuilder::Finalize() {

  // use a POT to determine the max relative pick offset
  detector::PickOffsetTable pot{arrival_picks_};

  // initialization time
  Core::TimeSpan po{pot.pick_offset().value_or(0)};
  if (po) {
    using pair_type = TemplateProcessorConfigs::value_type;
    const auto max{std::max_element(
        std::begin(processor_configs_), std::end(processor_configs_),
        [](const pair_type &lhs, const pair_type &rhs) {
          return lhs.second.processor->init_time() <
                 rhs.second.processor->init_time();
        })};

    product_->init_time_ = (max->second.processor->init_time() > po
                                ? max->second.processor->init_time()
                                : po);

  } else if (pot.size()) {
    product_->init_time_ =
        processor_configs_.cbegin()->second.processor->init_time();
  } else {
    product_->disable();
  }

  const auto &cfg{product_->config_};
  product_->detector_.set_trigger_thresholds(cfg.trigger_on, cfg.trigger_off);
  if (cfg.trigger_duration >= 0) {
    product_->detector_.EnableTrigger(Core::TimeSpan{cfg.trigger_duration});
  }
  auto product{product_.get()};
  product_->detector_.set_result_callback(
      [product](const detector::Detector::Result &res) {
        product->StoreDetection(res);
      });
  if (cfg.arrival_offset_threshold < 0) {
    product_->detector_.set_arrival_offset_threshold(boost::none);
  } else {
    product_->detector_.set_arrival_offset_threshold(
        cfg.arrival_offset_threshold);
  }

  for (auto &proc_config_pair : processor_configs_) {
    const auto &stream_id{proc_config_pair.first};
    auto &proc_config{proc_config_pair.second};

    // initialize buffer
    auto &buf{product_->stream_configs_[stream_id].stream_buffer};
    buf = std::make_shared<RingBuffer>(
        product_->init_time() * settings::kBufferMultiplicator, 0);

    const auto &meta{proc_config.metadata};
    boost::optional<std::string> phase_hint;
    try {
      phase_hint = meta.pick->phaseHint();
    } catch (Core::ValueException &e) {
    }
    // initialize detection processing strategy
    product_->detector_.Register(
        std::move(proc_config.processor), buf, stream_id,
        detector::Arrival{
            {meta.pick->time().value(), meta.pick->waveformID(), phase_hint,
             meta.pick->time().value() - product_->origin_->time().value()},
            meta.arrival->phase(),
            meta.arrival->weight(),
        },
        meta.pick_offset,
        detector::Detector::SensorLocation{
            meta.sensor_location->latitude(), meta.sensor_location->longitude(),
            meta.sensor_location->station()->publicID()});
  }
}

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

} // namespace detect
} // namespace Seiscomp
