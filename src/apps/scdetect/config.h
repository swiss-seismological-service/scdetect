#ifndef SCDETECT_APPS_SCDETECT_CONFIG_H_
#define SCDETECT_APPS_SCDETECT_CONFIG_H_

#include <seiscomp/core/datetime.h>

#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <initializer_list>
#include <string>
#include <unordered_map>
#include <vector>

#include "detector/arrival.h"
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

}  // namespace config

struct StreamConfig {
  // Stream related template configuration
  struct TemplateStreamConfig {
    std::string phase{"Pg"};

    double wfStart{-2};
    double wfEnd{2};

    // Defines a template specific waveform stream id
    std::string wfStreamId;
    // Defines a template specific filter
    boost::optional<std::string> filter;
  };
  StreamConfig();
  StreamConfig(const std::string &wfStreamId, const std::string &filter,
               const double initTime,
               const TemplateStreamConfig &templateConfig,
               const std::string &templateId = "");
  StreamConfig(const boost::property_tree::ptree &pt,
               const StreamConfig &defaults);

  bool isValid() const;

  // Template processor identifier
  std::string templateId{utils::createUUID()};

  std::string wfStreamId;

  double initTime{60};
  // Defines the processing specific filter
  boost::optional<std::string> filter;
  // Defines a stream specific merging threshold for using cross-correlation
  // results during the linking procedure
  boost::optional<double> mergingThreshold;
  // Defines the processing specific target sampling frequency, which might
  // force resampling the data to be processed
  boost::optional<double> targetSamplingFrequency;

  TemplateStreamConfig templateConfig;
};

struct DetectorConfig {
  // The default threshold to trigger the detector
  // - xcorr trigger thresholds [-1,1]
  double triggerOn{0.85L};
  // The default threshold to emit a detection once the detector is triggered
  // - Only has an effect if trigger duration is enabled, i.e. if
  // `triggerDuration` > 0
  // - xcorr trigger thresholds [-1,1]
  double triggerOff{0.65L};
  // The duration of a trigger
  // - setting a negative value disables the detector's trigger facilities
  double triggerDuration{-1};

  // The time correction in seconds to apply when an origin is going to be
  // emitted.
  double timeCorrection{0};

  // Flag indicating whether the detector is enabled (true) or disabled
  // (false).
  bool enabled{true};

  // Flag indicating whether to interpolate gaps linearly. Valid for `gaps <=
  // gapTolerance`.
  bool gapInterpolation{false};
  // Threshold in seconds to recognize a gap
  double gapThreshold{0.1};
  // Maximum gap length in seconds to tolerate and to be handled
  double gapTolerance{4.5};
  // Maximum data latency in seconds tolerated with regards to `NOW`
  double maximumLatency{10};

  // Maximum inter arrival offset threshold in seconds to tolerate when
  // associating an arrival to an event
  // - the threshold is only validated for multi-stream detectors
  // - setting a negative value disables the arrival offset validation
  double arrivalOffsetThreshold{2.0e-6};
  // Defines the minimum number of arrivals which must be part of an event to be
  // declared as a detection
  // - setting a negative value disables the validation i.e. all arrivals must
  // be available (default)
  int minArrivals{-1};
  // Defines the linker's merging strategy which may lead to dropping template
  // waveform processor results if not fulfilling the merging strategy's
  // criteria
  std::string mergingStrategy{"greaterEqualTriggerOnThreshold"};

  // Defines the chunk size in seconds which is used to feed data to template
  // waveform processors
  // - setting a negative value forces a default chunk size of 10s
  double chunkSize{10};

  bool isValid(size_t numStreamConfigs) const;
};

struct PublishConfig {
  // Indicates whether to append *detected arrivals* to declared origins
  bool createArrivals{false};
  // Indicates whether to append *theoretical template arrivals* to declared
  // origins
  bool createTemplateArrivals{false};

  // The origin method identifier
  std::string originMethodId{"DETECT"};

  std::vector<detector::Arrival> theoreticalTemplateArrivals;
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
                 const DetectorConfig &detectorDefaults,
                 const StreamConfig &streamDefaults,
                 const PublishConfig &publishDefaults);

  std::string detectorId() const;
  std::string originId() const;
  DetectorConfig detectorConfig() const;
  PublishConfig publishConfig() const;

  size_type size() const noexcept { return _streamConfigs.size(); }
  reference &at(const std::string &stream_id);

  iterator begin() { return _streamConfigs.begin(); }
  iterator end() { return _streamConfigs.end(); }
  const_iterator begin() const { return _streamConfigs.begin(); }
  const_iterator end() const { return _streamConfigs.end(); }
  const_iterator cbegin() const { return _streamConfigs.cbegin(); }
  const_iterator cend() const { return _streamConfigs.cend(); }

 private:
  std::string _detectorId{utils::createUUID()};

  std::string _originId;

  PublishConfig _publishConfig;

  DetectorConfig _detectorConfig;

  StreamConfigs _streamConfigs;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_CONFIG_H_
