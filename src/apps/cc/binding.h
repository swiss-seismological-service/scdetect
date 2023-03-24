#ifndef SCDETECT_APPS_CC_BINDING_H_
#define SCDETECT_APPS_CC_BINDING_H_

#include <seiscomp/config/config.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/datamodel/configmodule.h>
#include <seiscomp/processing/processor.h>
#include <seiscomp/utils/keyvalues.h>

#include <boost/optional/optional.hpp>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace binding {

struct StreamConfig {
  struct DeconvolutionConfig {
    // Indicates whether deconvolution is enabled `true` or not `false`
    bool enabled{true};
    // Taper length in seconds when deconvolving the data
    double responseTaperLength{5};
    // Defines the end of the left-hand side cosine-taper in Hz applied to the
    // frequency spectrum. I.e. the spectrum is tapered between 0Hz and
    // `minimumResponseTaperFrequency`. A value less than or equal to zero
    // disables left-hand side tapering.
    double minimumResponseTaperFrequency{0.00833333};  // 120 seconds
    // Defines the beginning of the right-hand side cosine-taper in Hz applied
    // to the frequency spectrum. I.e. the spectrum is tapered between
    // `maximumResponseTaperFrequency` and the Nyquist frequency. A value less
    // than or equal to zero disables left-hand side tapering.
    double maximumResponseTaperFrequency{0};

    // Returns a string with debug information
    std::string debugString() const;
  };

  DeconvolutionConfig deconvolutionConfig;
};

struct SensorLocationConfig {
  // Amplitude processing configuration
  struct AmplitudeProcessingConfig {
    struct MLx {
      // The filter string identifier used for amplitude calculation
      boost::optional<std::string> filter{"BW_BP(4,10,30)"};
      // The filter's initialization time in seconds
      Core::TimeSpan initTime{60};
      // Defines the saturation threshold for the saturation check. If unset, no
      // saturation check is performed.
      boost::optional<double> saturationThreshold;
    };

    struct MRelative {
      // The filter string identifier used for amplitude calculation
      boost::optional<std::string> filter;
      // The filter's initialization time in seconds
      Core::TimeSpan initTime{60};
      // Defines the saturation threshold for the saturation check. If unset, no
      // saturation check is performed.
      boost::optional<double> saturationThreshold;
    };

    MLx mlx;
    MRelative mrelative;

    // Defines the amplitude types to be computed with regard to the sensor
    // location
    std::vector<std::string> amplitudeTypes{"MRelative"};
    // Indicates whether the amplitude calculation is enabled (`true`) or
    // disabled (`false`)
    bool enabled{true};
  };

  // Magnitude processing configuration
  struct MagnitudeProcessingConfig {
    struct MRelative {
      // Defines whether to use the network magnitude (`true`) or the station
      // magnitude (`false`) for magnitude estimation
      bool useNetworkMagnitude{true};
    };

    MRelative mrelative;
    // Defines the magnitude types to be computed with regard to the sensor
    // location
    std::vector<std::string> magnitudeTypes{"MRelative"};
    // Indicates whether the sensor location is enabled (`true`) or disabled
    // (`false`)
    bool enabled{true};
  };

  // Returns the Stream configuration for `chaCode`
  const StreamConfig &at(const std::string &chaCode) const;

  const StreamConfig &matchWildcards(const std::string &chaCode) const;

  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;
  StreamConfigs streamConfigs;

  AmplitudeProcessingConfig amplitudeProcessingConfig;
  MagnitudeProcessingConfig magnitudeProcessingConfig;

  // The default stream configuration
  StreamConfig _defaultStreamConfig;
};

// A container for station configuration
//
// - Maintains configuration on sensor location granularity.
class StationConfig {
  using LocationCode = std::string;
  using ChannelCode = std::string;
  using Key = std::pair<LocationCode, ChannelCode>;
  using ConfigMap = std::map<Key, SensorLocationConfig>;

 public:
  using const_iterator = ConfigMap::const_iterator;
  const_iterator begin() const;
  const_iterator end() const;

  const SensorLocationConfig &at(const std::string &locCode,
                                 const std::string &chaCode) const;

 private:
  const SensorLocationConfig &matchWildcards(const std::string &locCode,
                                             const std::string &chaCode) const;

  ConfigMap _sensorLocationConfigs;

  Util::KeyValuesCPtr _parameters;

  friend class Bindings;
};

// A container for binding configuration
//
// - For further information please refer to
// https://www.seiscomp.de/doc/base/concepts/configuration.html#global-bindings-config
class Bindings {
  using NetworkCode = std::string;
  using StationCode = std::string;
  using Key = std::pair<NetworkCode, StationCode>;
  using ConfigMap = std::map<Key, StationConfig>;

 public:
  // Generic wildcard character for binding association
  static const char wildcardZeroToManyChar;
  static const char wildcardSingleChar;

  using const_iterator = ConfigMap::const_iterator;

  const_iterator begin() const;
  const_iterator end() const;

  // Returns the `StationConfig` regarding `netCode` and `staCode`
  const StationConfig &at(const std::string &netCode,
                          const std::string &staCode) const;

  // Returns the `SensorLocationConfig regarding `netCode`, `staCode`,
  // `locCode` and `chaCode`.
  const SensorLocationConfig &at(const std::string &netCode,
                                 const std::string &staCode,
                                 const std::string &locCode,
                                 const std::string &chaCode) const;

  // Loads the binding configuration from both the (file based) `moduleConfig`
  // and the `configModule`. The `configModule` is loaded taking the `setupId`
  // name into account.
  void load(const Seiscomp::Config::Config *moduleConfig,
            const DataModel::ConfigModule *configModule,
            const std::string &setupId);

  // Sets the default sensor location configuration
  void setDefault(const SensorLocationConfig &defaultSensorLocationConfig);

 protected:
  // Load a `StationConfig` while taking both the `parameterSet` (i.e. the
  // station specific bindings) and `moduleConfig` (i.e. the module
  // configuration) into account.
  //
  // - Note that the configuration provided by `parameterSet` overrides the
  // configuration provided by `moduleConfig`.
  const StationConfig &load(const Seiscomp::Config::Config *moduleConfig,
                            DataModel::ParameterSet *parameterSet,
                            const std::string &configModuleId,
                            const std::string &netCode,
                            const std::string &staCode);

 private:
  // The default sensor location configuration
  SensorLocationConfig _defaultSensorLocationConfig;

  ConfigMap _stationConfigs;
};

// Parse the saturation threshold from `settings` identified by `parameter`
//
// - returns `boost::none` if no parameter could be found
boost::optional<double> parseSaturationThreshold(
    const Processing::Settings &settings, const std::string &parameter);

namespace detail {
// Savely sets the `filter` at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setFilter(const std::string &filter, std::string &storageLocation);
// Savely sets the `filter` at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setFilter(const boost::optional<std::string> &filter,
               boost::optional<std::string> &storageLocation);

// Savely sets the initialization time at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setInitTime(const Core::TimeSpan &initTime,
                 Core::TimeSpan &storageLocation);
// Savely sets the filter (at `storageLocationFilter`) and initialization time
// (at `storageLocationInitTime` )from `settings` identified by
// `parameterFilter` and `parameterInitTime`
//
// - if a lookup wasn't successful values are taken from `defaultFilter` and
// `defaultInitTime`, respectively
// - throws a `ValueException` if the values are invalid
void setFilter(const Processing::Settings &settings,
               const std::string &parameterFilter,
               const std::string &parameterInitTime,
               const boost::optional<std::string> &defaultFilter,
               const Core::TimeSpan &defaultInitTime,
               boost::optional<std::string> &storageLocationFilter,
               Core::TimeSpan &storageLocationInitTime);
// Savely sets the saturation threshold (at `storageLocation`) from `settings`
// identified by `parameter`
//
// - throws a `ValueException` if the value is invalid
void setSaturationThreshold(const Processing::Settings &settings,
                            const std::string &parameter,
                            boost::optional<double> &storageLocation);
// Savely sets the response taper length at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setResponseTaperLength(double length, double &storageLocation);
// Savely sets the response taper length from `settings` identified by
// `parameter` at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setResponseTaperLength(const Processing::Settings &settings,
                            const std::string &parameter,
                            double &storageLocation,
                            const boost::optional<double> & = boost::none);
// Savely sets the response taper frequency at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setResponseTaperFrequency(double f, double &storageLocation);
// Savely sets the minimum response taper frequency from `settings`
// identified by `parameter` at `storageLocation`
//
// - throws a `ValueException` if the value is invalid
void setResponseTaperFrequency(
    const Processing::Settings &settings, const std::string &parameter,
    double &storageLocation,
    const boost::optional<double> &defaultValue = boost::none);

void load(const Processing::Settings &settings,
          const std::string &parameterPrefix, StreamConfig &storageLocation,
          const StreamConfig &defaults);
void load(const Processing::Settings &settings,
          const std::string &parameterPrefix,
          SensorLocationConfig::AmplitudeProcessingConfig &storageLocation,
          const SensorLocationConfig::AmplitudeProcessingConfig &defaults);
void load(const Processing::Settings &settings,
          const std::string &parameterPrefix,
          SensorLocationConfig::MagnitudeProcessingConfig &storageLocation,
          const SensorLocationConfig::MagnitudeProcessingConfig &defaults);
void validate(SensorLocationConfig &config);

std::string removeDuplicateChar(const std::string &str, char c);

void replaceWildcardChars(std::string &str);

}  // namespace detail

}  // namespace binding
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_BINDING_H_
