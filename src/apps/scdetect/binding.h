#ifndef SCDETECT_APPS_SCDETECT_BINDING_H_
#define SCDETECT_APPS_SCDETECT_BINDING_H_

#include <seiscomp/config/config.h>
#include <seiscomp/datamodel/configmodule.h>
#include <seiscomp/processing/processor.h>
#include <seiscomp/utils/keyvalues.h>

#include <boost/optional/optional.hpp>
#include <map>
#include <string>
#include <unordered_map>

#include "amplitudeprocessor.h"

namespace Seiscomp {
namespace detect {
namespace binding {

// Parse the saturation threshold from `settings` identified by `parameter`
//
// - returns `boost::none` if no parameter could be found
boost::optional<double> parseSaturationThreshold(
    const Processing::Settings &settings, const std::string &parameter);

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

    explicit operator AmplitudeProcessor::DeconvolutionConfig() const;

    // Savely sets the response taper length
    //
    // - throws a `ValueException` if the value is invalid
    void setResponseTaperLength(double length);
    // Savely sets the response taper length from `settings` identified by
    // `parameter`
    //
    // - throws a `ValueException` if the value is invalid
    void setResponseTaperLength(const Processing::Settings &settings,
                                const std::string &parameter,
                                const DeconvolutionConfig &defaultConfig);
    // Savely sets the minimum response taper frequency
    //
    // - throws a `ValueException` if the value is invalid
    void setMinimumResponseTaperFrequency(double f);
    // Savely sets the minimum response taper frequency from `settings`
    // identified by `parameter`
    //
    // - throws a `ValueException` if the value is invalid
    void setMinimumResponseTaperFrequency(
        const Processing::Settings &settings, const std::string &parameter,
        const DeconvolutionConfig &defaultConfig);
    // Savely sets the maximum response taper frequency
    //
    // - throws a `ValueException` if the value is invalid
    void setMaximumResponseTaperFrequency(double f);
    // Savely sets the maximum response taper frequency from `settings`
    // identified by `parameter`
    //
    // - throws a `ValueException` if the value is invalid
    void setMaximumResponseTaperFrequency(
        const Processing::Settings &settings, const std::string &parameter,
        const DeconvolutionConfig &defaultConfig);
  };

  DeconvolutionConfig deconvolutionConfig;
};

struct SensorLocationConfig {
  // Amplitude processing configuration
  struct AmplitudeProcessingConfig {
    // Indicates whether the stream is enabled (`true`) or disabled (`false`)
    bool enabled{true};
    // The filter string identifier used for amplitude calculation
    std::string filter;
    // The filter's initialization time in seconds
    double initTime{60};
    // Defines the saturation threshold for the saturation check. If unset, no
    // saturation check is performed.
    boost::optional<double> saturationThreshold;

    // Savely sets the filter
    //
    // - throws a `ValueException` if the value is invalid
    void setFilter(const std::string &filter);
    // Savely sets the initialization time
    //
    // - throws a `ValueException` if the value is invalid
    void setInitTime(double initTime);
    // Savely sets the filter and initialization time from `settings` identified
    // by `parameterFilter` and `parameterInitTime`
    //
    // - if a lookup wasn't successful values are taken from `defaultConfig`
    // - throws a `ValueException` if the values are invalid
    void setFilter(const Processing::Settings &settings,
                   const std::string &parameterFilter,
                   const std::string &parameterInitTime,
                   const AmplitudeProcessingConfig &defaultConfig);
    // Savely sets the saturation threshold from `settings` identified by
    // `parameter`
    //
    // - throws a `ValueException` if the value is invalid
    void setSaturationThreshold(const Processing::Settings &settings,
                                const std::string &parameter);
  };

  // Returns the Stream configuration for `chaCode`
  const StreamConfig &at(const std::string &chaCode) const;

  AmplitudeProcessingConfig amplitudeProcessingConfig;

  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;
  StreamConfigs streamConfigs;
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

}  // namespace binding
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_BINDING_H_
