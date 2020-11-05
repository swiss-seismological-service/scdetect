#ifndef SCDETECT_APPS_SCDETECT_CONFIG_H_
#define SCDETECT_APPS_SCDETECT_CONFIG_H_

#include <boost/property_tree/ptree_fwd.hpp>
#include <initializer_list>
#include <string>
#include <unordered_map>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <seiscomp/core/datetime.h>

namespace Seiscomp {
namespace detect {

struct StreamConfig {

  // Stream related template configuration
  struct TemplateStreamConfig {
    std::string phase{""};

    double wf_start{-2};
    double wf_end{2};
  };
  StreamConfig();
  StreamConfig(const std::string &wf_stream_id, const std::string &filter,
               const double init_time, const bool sensitivity_correction,
               const TemplateStreamConfig &template_config);
  StreamConfig(const boost::property_tree::ptree &pt,
               const StreamConfig &defaults);

  bool IsValid() const;

  std::string wf_stream_id{""};

  double init_time{60};
  std::string filter{""};

  // Defines whether sensitivity correction (gain) is applied or not in advance
  // to filter the data.
  bool sensitivity_correction{true};

  // Stream related template configuration
  TemplateStreamConfig template_config;
};

struct DetectorConfig {
  // The default threshold to trigger - xcorr trigger thresholds [0,1]
  double trigger_on{0.7L};
  // The default threshold to enabling triggering, again - xcorr trigger
  // thresholds [0,1]
  double trigger_off{0.5L};
  // The dead-time in seconds after a pick has been detected
  double trigger_dead_time{30};
  // The duration of a trigger, negative value = disabled
  double trigger_duration{-1};

  // The time correction in seconds to apply when an origin is going to be
  // emitted.
  double time_correction{0};

  // Flag indicating whether the detector is enabled (true) or disabled
  // (false).
  bool enabled{true};

  // Flag indicating whether to interpolate gaps linearly. Valid for gaps <=
  // gap_tolerance.
  bool gap_interpolation{false};
  // Maximum gap length in seconds to handle.
  double gap_tolerance{4.5};
  // Maximum data latency a in seconds tolerated
  double maximum_latency{10};

  // Buffer size (in seconds)
  /* int buffer_size = settings::kDefaultBufferSize; */
  // Processing interval in seconds
  /* double processing_interval = settings::kDefaultProcessingInterval; */

  bool IsValid() const;
};

// Container for StreamConfig
using StreamConfigs = std::unordered_map<std::string, StreamConfig>;

class TemplateConfig {
  using StreamSets = std::vector<StreamConfigs>;

public:
  using size_type = StreamSets::size_type;
  using value_type = StreamSets::value_type;
  using reference = value_type &;
  using iterator = StreamSets::iterator;
  using const_iterator = StreamSets::const_iterator;

  TemplateConfig(const boost::property_tree::ptree &pt,
                 const DetectorConfig &detector_defaults,
                 const StreamConfig &stream_defaults);

  const std::string origin_id() const;
  const std::string phase(const size_t priority = 0) const;
  const DetectorConfig detector_config() const;

  size_type size() const noexcept { return stream_sets_.size(); }
  reference &at(size_t priority);

  iterator begin() { return stream_sets_.begin(); }
  iterator end() { return stream_sets_.end(); }
  const_iterator begin() const { return stream_sets_.begin(); }
  const_iterator end() const { return stream_sets_.end(); }
  const_iterator cbegin() const { return stream_sets_.cbegin(); }
  const_iterator cend() const { return stream_sets_.cend(); }

private:
  std::string origin_id_;
  DetectorConfig detector_config_;

  StreamSets stream_sets_;

  // TODO(damb): Allow additional stream sets to be allowed; most probably
  // this requires adopting the underlying datastructure of StreamConfigs since
  // the mapping between two StreamConfig objects must be unambiguous
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_CONFIG_H_
