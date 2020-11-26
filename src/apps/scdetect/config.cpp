#include "config.h"

#include <utility>
#include <vector>

#include <boost/property_tree/exceptions.hpp>

#include "exception.h"
#include "utils.h"
#include "version.h"

namespace Seiscomp {
namespace detect {

StreamConfig::StreamConfig() {}

StreamConfig::StreamConfig(const std::string &wf_stream_id,
                           const std::string &filter, const double init_time,
                           const bool sensitivity_correction,
                           const TemplateStreamConfig &template_config)
    : wf_stream_id(wf_stream_id), init_time(init_time), filter(filter),
      sensitivity_correction(sensitivity_correction),
      template_config(template_config) {}

StreamConfig::StreamConfig(const boost::property_tree::ptree &pt,
                           const StreamConfig &defaults)
    : wf_stream_id(pt.get<std::string>("waveformId")),
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
          utils::ValidatePhase(template_config.phase) &&
          utils::IsGeZero(init_time));
}

bool DetectorConfig::IsValid() const {
  if (!utils::ValidateXCorrThreshold(trigger_on) ||
      !utils::ValidateXCorrThreshold(trigger_off) ||
      (gap_interpolation && !utils::IsGeZero(gap_tolerance))) {
    return false;
  }
  return true;
}

TemplateConfig::TemplateConfig(const boost::property_tree::ptree &pt,
                               const DetectorConfig &detector_defaults,
                               const StreamConfig &stream_defaults)
    : origin_id_(pt.get<std::string>("originId")) {
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
  detector_config_.gap_tolerance =
      pt.get<bool>("gapTolerance", detector_defaults.gap_tolerance);
  // TODO(damb): Should we specify the detector's init time based on the init
  // times of the underlying Template processors?
  detector_config_.maximum_latency =
      pt.get<double>("maximumLatency", detector_defaults.maximum_latency);

  if (!detector_config_.IsValid()) {
    throw std::invalid_argument(
        "Invalid template specific detector configuration.");
  }

  // initialize stream configs
  for (const auto &stream_set_pair : pt.find("streams")->second) {
    StreamConfigs stream_configs;
    for (const auto &stream_config_pair : stream_set_pair.second) {

      std::string wf_id;
      try {
        StreamConfig stream_config{stream_config_pair.second, stream_defaults};
        stream_configs.emplace(stream_config.wf_stream_id, stream_config);
        wf_id = stream_config.wf_stream_id;
      } catch (boost::property_tree::ptree_error &e) {
        SEISCOMP_WARNING("Exception while parsing stream_config: %s", e.what());
        continue;
      }

      if (!stream_configs[wf_id].IsValid()) {
        SEISCOMP_WARNING("Exception while parsing stream_config: Invalid "
                         "stream configuration for stream: %s",
                         wf_id.c_str());
        continue;
      }
    }
    stream_sets_.push_back(stream_configs);
  }
}

const std::string TemplateConfig::origin_id() const { return origin_id_; }

const std::string TemplateConfig::phase(const size_t priority) const {
  if (priority >= size()) {
    return "";
  }

  std::string p{
      std::begin(stream_sets_[priority])->second.template_config.phase};
  for (const auto &stream_config_pair : stream_sets_[priority]) {
    if (stream_config_pair.second.template_config.phase != p)
      return "";
  }
  return p;
}

const DetectorConfig TemplateConfig::detector_config() const {
  return detector_config_;
}

TemplateConfig::reference TemplateConfig::at(size_t priority) {
  return stream_sets_.at(priority);
}

} // namespace detect
} // namespace Seiscomp
