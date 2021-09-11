#ifndef SCDETECT_APPS_SCDETECT_BINDING_H_
#define SCDETECT_APPS_SCDETECT_BINDING_H_

#include <seiscomp/config/config.h>
#include <seiscomp/datamodel/configmodule.h>
#include <seiscomp/utils/keyvalues.h>

#include <map>
#include <string>

#include "amplitudeprocessor.h"

namespace Seiscomp {
namespace detect {
namespace binding {

struct SensorLocationConfig {
  // Amplitude processing configuration
  struct AmplitudeProcessingConfig {
    // Indicates whether the stream is enabled (`true`) or disabled (`false`)
    bool enabled{true};
    // The filter string identifier used for amplitude calculation
    std::string filter;
    // The filter's initialization time in seconds
    double initTime{60};

    bool isValid() const;
  };

  AmplitudeProcessingConfig amplitudeProcessingConfig;
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
