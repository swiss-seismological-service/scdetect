#include "config.h"

#include <utility>
#include <vector>

#include <boost/property_tree/exceptions.hpp>

#include "exception.h"
#include "log.h"
#include "settings.h"
#include "utils.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {

namespace config {

BaseException::BaseException() : Exception("base config exception") {}

ParserException::ParserException()
    : BaseException{"error while parsing configuration"} {}

} // namespace config

StreamConfig::StreamConfig() {}

StreamConfig::StreamConfig(const std::string &wf_stream_id,
                           const std::string &filter, const double init_time,
                           const bool sensitivity_correction,
                           const TemplateStreamConfig &template_config,
                           const std::string &template_id)
    : wf_stream_id(wf_stream_id), init_time(init_time), filter(filter),
      sensitivity_correction(sensitivity_correction),
      template_config(template_config) {}

StreamConfig::StreamConfig(const boost::property_tree::ptree &pt,
                           const StreamConfig &defaults)
    : template_id{pt.get<std::string>("templateId", utils::CreateUUID())

      },
      wf_stream_id(pt.get<std::string>("waveformId")),
      init_time(pt.get<double>("initTime", defaults.init_time)),
      filter(pt.get<std::string>("filter", defaults.filter)),
      sensitivity_correction(pt.get<bool>("sensitivityCorrection",
                                          defaults.sensitivity_correction)) {
  template_config.phase =
      pt.get<std::string>("templatePhase", defaults.template_config.phase);
  template_config.wf_start = pt.get<double>("templateWaveformStart",
                                            defaults.template_config.wf_start);
  template_config.wf_end =
      pt.get<double>("templateWaveformEnd", defaults.template_config.wf_end);
  template_config.wf_stream_id =
      pt.get<std::string>("templateWaveformId", wf_stream_id);
  template_config.filter =
      pt.get<std::string>("templateFilter", defaults.template_config.filter);
}

bool StreamConfig::IsValid() const {
  bool retval{true};
  try {
    retval = utils::WaveformStreamID{wf_stream_id}.IsValid();
  } catch (ValueException &e) {
    return false;
  }
  try {
    retval = utils::WaveformStreamID{template_config.wf_stream_id}.IsValid();
  } catch (ValueException &e) {
    return false;
  }

  return (retval && template_config.wf_start < template_config.wf_end &&
          !template_config.phase.empty() && utils::IsGeZero(init_time));
}

bool DetectorConfig::IsValid(size_t num_stream_configs) const {
  return (config::ValidateXCorrThreshold(trigger_on) &&
          config::ValidateXCorrThreshold(trigger_off) &&
          (!gap_interpolation ||
           (gap_interpolation && utils::IsGeZero(gap_threshold) &&
            utils::IsGeZero(gap_tolerance) && gap_threshold < gap_tolerance)) &&
          config::ValidateArrivalOffsetThreshold(arrival_offset_threshold) &&
          config::ValidateMinArrivals(min_arrivals,
                                      static_cast<int>(num_stream_configs)));
}

TemplateConfig::TemplateConfig(const boost::property_tree::ptree &pt,
                               const DetectorConfig &detector_defaults,
                               const StreamConfig &stream_defaults)
    : detector_id_{pt.get<std::string>("detectorId", utils::CreateUUID())},
      origin_id_(pt.get<std::string>("originId")) {
  detector_config_.trigger_on =
      pt.get<double>("triggerOnThreshold", detector_defaults.trigger_on);
  detector_config_.trigger_off =
      pt.get<double>("triggerOffThreshold", detector_defaults.trigger_off);
  detector_config_.trigger_duration =
      pt.get<double>("triggerDuration", detector_defaults.trigger_duration);
  detector_config_.time_correction =
      pt.get<double>("timeCorrection", detector_defaults.time_correction);
  detector_config_.gap_interpolation =
      pt.get<bool>("gapInterpolation", detector_defaults.gap_interpolation);
  detector_config_.gap_threshold =
      pt.get<double>("gapThreshold", detector_defaults.gap_threshold);
  detector_config_.gap_tolerance =
      pt.get<double>("gapTolerance", detector_defaults.gap_tolerance);
  // TODO(damb): Should we specify the detector's init time based on the init
  // times of the underlying Template processors?
  detector_config_.maximum_latency =
      pt.get<double>("maximumLatency", detector_defaults.maximum_latency);
  detector_config_.create_arrivals =
      pt.get<bool>("createArrivals", detector_defaults.create_arrivals);
  detector_config_.arrival_offset_threshold = pt.get<double>(
      "arrivalOffsetThreshold", detector_defaults.arrival_offset_threshold);
  detector_config_.min_arrivals =
      pt.get<int>("minimumArrivals", detector_defaults.min_arrivals);

  // patch stream defaults with detector config globals
  auto patched_stream_defaults{stream_defaults};
  patched_stream_defaults.init_time =
      pt.get<double>("initTime", stream_defaults.init_time);
  patched_stream_defaults.filter =
      pt.get<std::string>("filter", stream_defaults.filter);
  patched_stream_defaults.sensitivity_correction = pt.get<bool>(
      "sensitivityCorrection", stream_defaults.sensitivity_correction);
  patched_stream_defaults.template_config.phase = pt.get<std::string>(
      "templatePhase", stream_defaults.template_config.phase);
  patched_stream_defaults.template_config.wf_start = pt.get<double>(
      "templateWaveformStart", stream_defaults.template_config.wf_start);
  patched_stream_defaults.template_config.wf_end = pt.get<double>(
      "templateWaveformEnd", stream_defaults.template_config.wf_end);
  patched_stream_defaults.template_config.filter = pt.get<std::string>(
      "templateFilter", stream_defaults.template_config.filter);

  // initialize stream configs
  for (const auto &stream_config_pair : pt.find("streams")->second) {
    const auto &sc{stream_config_pair.second};

    std::string wf_id;
    try {
      StreamConfig stream_config{sc, patched_stream_defaults};
      stream_configs_.emplace(stream_config.wf_stream_id, stream_config);
      wf_id = stream_config.wf_stream_id;
    } catch (boost::property_tree::ptree_error &e) {
      throw config::ParserException{
          std::string{"Exception while parsing stream config: "} + e.what()};
    }

    if (!stream_configs_[wf_id].IsValid()) {
      throw config::ParserException{
          std::string{"Exception while parsing stream_config: Invalid "
                      "stream configuration for stream: "} +
          wf_id};
    }
  }

  const auto max_arrivals{stream_configs_.size()};
  if (detector_config_.min_arrivals > static_cast<int>(max_arrivals)) {
    SCDETECT_LOG_WARNING_TAGGED(
        detector_id_,
        "Configured number of minimum arrivals exceets number of configured "
        "streams (%d > %d). Resetting.",
        detector_config_.min_arrivals, max_arrivals);
    detector_config_.min_arrivals = max_arrivals;
  }

  if (!detector_config_.IsValid(max_arrivals)) {
    throw config::ParserException{
        "Invalid template specific detector configuration"};
  }
}

const std::string TemplateConfig::detector_id() const { return detector_id_; }

const std::string TemplateConfig::origin_id() const { return origin_id_; }

const DetectorConfig TemplateConfig::detector_config() const {
  return detector_config_;
}

TemplateConfig::reference TemplateConfig::at(const std::string &stream_id) {
  return stream_configs_.at(stream_id);
}

} // namespace detect
} // namespace Seiscomp
