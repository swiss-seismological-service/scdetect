#ifndef SCDETECT_APPS_SCDETECT_CONFIG_H_
#define SCDETECT_APPS_SCDETECT_CONFIG_H_

#include <initializer_list>
#include <string>
#include <unordered_map>

#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <seiscomp/core/datetime.h>

#include "exception.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {

namespace config {

class BaseException : public Exception {
public:
  using Exception::Exception;
  BaseException();
};

class ParserException : public BaseException {
public:
  using BaseException::BaseException;
  ParserException();
};

} // namespace config

struct StreamConfig {

  // Stream related template configuration
  struct TemplateStreamConfig {
    std::string phase{"Pg"};

    double wf_start{-2};
    double wf_end{2};

    // Defines a template specific waveform stream id
    std::string wf_stream_id;
    // Defines a template specific filter
    boost::optional<std::string> filter;
  };
  StreamConfig();
  StreamConfig(const std::string &wf_stream_id, const std::string &filter,
               const double init_time,
               const TemplateStreamConfig &template_config,
               const std::string &template_id = "");
  StreamConfig(const boost::property_tree::ptree &pt,
               const StreamConfig &defaults);

  bool IsValid() const;

  // Template processor identifier
  std::string template_id{utils::CreateUUID()};

  std::string wf_stream_id;

  double init_time{60};
  // Defines the processing specific filter
  boost::optional<std::string> filter;

  TemplateStreamConfig template_config;
};

struct DetectorConfig {
  // The default threshold to trigger the detector
  // - xcorr trigger thresholds [-1,1]
  double trigger_on{0.85L};
  // The default threshold to emit a detection once the detector is triggered
  // - Only has an effect if trigger duration is enabled, i.e. if
  // `trigger_duration` > 0
  // - xcorr trigger thresholds [-1,1]
  double trigger_off{0.65L};
  // The duration of a trigger
  // - setting a negative value disables the detector's trigger facilities
  double trigger_duration{-1};

  // The time correction in seconds to apply when an origin is going to be
  // emitted.
  double time_correction{0};

  // Flag indicating whether the detector is enabled (true) or disabled
  // (false).
  bool enabled{true};

  // Flag indicating whether to interpolate gaps linearly. Valid for `gaps <=
  // gap_tolerance`.
  bool gap_interpolation{false};
  // Threshold in seconds to recognize a gap
  double gap_threshold{0.1};
  // Maximum gap length in seconds to tolerate and to be handled
  double gap_tolerance{4.5};
  // Maximum data latency in seconds tolerated with regards to `NOW`
  double maximum_latency{10};

  // Flag indicating whether to compute and associate detected arrivals
  bool create_arrivals{false};
  // Flag indicating whether to associate template arrivals with a detection
  bool create_template_arrivals{false};

  // Maximum inter arrival offset threshold in seconds to tolerate when
  // associating an arrival to an event
  // - the threshold is only validated for multi-stream detectors
  // - setting a negative value disables the arrival offset validation
  double arrival_offset_threshold{2.0e-6};
  // Defines the minimum number of arrivals which must be part of an event to be
  // declared as a detection
  // - setting a negative value disables the validation i.e. all arrivals must
  // be available (default)
  int min_arrivals{-1};

  // Processing interval in seconds
  /* double processing_interval = settings::kDefaultProcessingInterval; */

  bool IsValid(size_t num_stream_configs) const;
};

class TemplateConfig {
  // Container for StreamConfig
  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;

public:
  using size_type = StreamConfigs::size_type;
  using value_type = StreamConfigs::value_type;
  using reference = StreamConfigs::mapped_type &;
  using iterator = StreamConfigs::iterator;
  using const_iterator = StreamConfigs::const_iterator;

  TemplateConfig(const boost::property_tree::ptree &pt,
                 const DetectorConfig &detector_defaults,
                 const StreamConfig &stream_defaults);

  const std::string detector_id() const;
  const std::string origin_id() const;
  const DetectorConfig detector_config() const;

  size_type size() const noexcept { return stream_configs_.size(); }
  reference &at(const std::string &stream_id);

  iterator begin() { return stream_configs_.begin(); }
  iterator end() { return stream_configs_.end(); }
  const_iterator begin() const { return stream_configs_.begin(); }
  const_iterator end() const { return stream_configs_.end(); }
  const_iterator cbegin() const { return stream_configs_.cbegin(); }
  const_iterator cend() const { return stream_configs_.cend(); }

private:
  std::string detector_id_{utils::CreateUUID()};

  std::string origin_id_;
  DetectorConfig detector_config_;

  StreamConfigs stream_configs_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_CONFIG_H_
