#include "binding.h"

#include <seiscomp/core/strings.h>
#include <seiscomp/datamodel/configstation.h>
#include <seiscomp/datamodel/parameter.h>
#include <seiscomp/datamodel/parameterset.h>
#include <seiscomp/datamodel/setup.h>
#include <seiscomp/datamodel/utils.h>
#include <seiscomp/processing/processor.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <string>
#include <vector>

#include "log.h"
#include "settings.h"
#include "utils.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {
namespace binding {

bool SensorLocationConfig::AmplitudeProcessingConfig::isValid() const {
  std::string err;
  return (filter.empty() || config::validateFilter(filter, err)) &&
         utils::isGeZero(initTime);
}

StationConfig::const_iterator StationConfig::begin() const {
  return _sensorLocationConfigs.begin();
}

StationConfig::const_iterator StationConfig::end() const {
  return _sensorLocationConfigs.end();
}

const SensorLocationConfig &StationConfig::at(
    const std::string &locCode, const std::string &chaCode) const {
  return _sensorLocationConfigs.at({locCode, chaCode});
}

Bindings::const_iterator Bindings::begin() const {
  return _stationConfigs.begin();
}

Bindings::const_iterator Bindings::end() const { return _stationConfigs.end(); }

const StationConfig &Bindings::at(const std::string &netCode,
                                  const std::string &staCode) const {
  return _stationConfigs.at({netCode, staCode});
}

const SensorLocationConfig &Bindings::at(const std::string &netCode,
                                         const std::string &staCode,
                                         const std::string &locCode,
                                         const std::string &chaCode) const {
  return _stationConfigs.at({netCode, staCode}).at(locCode, chaCode);
}

void Bindings::load(const Seiscomp::Config::Config *moduleConfig,
                    const DataModel::ConfigModule *configModule,
                    const std::string &setupId) {
  if (!configModule) return;

  for (size_t j = 0; j < configModule->configStationCount(); ++j) {
    DataModel::ConfigStation *stationConfig{configModule->configStation(j)};
    DataModel::Setup *configSetup{
        DataModel::findSetup(stationConfig, setupId, false)};

    if (configSetup) {
      DataModel::ParameterSet *parameterSet{nullptr};
      try {
        parameterSet =
            DataModel::ParameterSet::Find(configSetup->parameterSetID());
      } catch (Core::ValueException &) {
      }

      if (!parameterSet) {
        SCDETECT_LOG_WARNING("Failed to find parameter set with id: %s",
                             configSetup->parameterSetID().c_str());
        continue;
      }

      load(moduleConfig, parameterSet, configModule->name(),
           stationConfig->networkCode(), stationConfig->stationCode());
    }
  }
}

void Bindings::setDefault(
    const SensorLocationConfig &defaultSensorLocationConfig) {
  _defaultSensorLocationConfig = defaultSensorLocationConfig;
}

const StationConfig &Bindings::load(
    const Seiscomp::Config::Config *moduleConfig,
    DataModel::ParameterSet *parameterSet, const std::string &configModuleId,
    const std::string &netCode, const std::string &staCode) {
  std::string locCode, chaCode;

  Util::KeyValuesPtr keys;
  if (parameterSet) {
    keys = utils::make_smart<Util::KeyValues>();
    keys->init(parameterSet);
  }

  // XXX(damb): merges module configuration with bindings
  Processing::Settings settings{configModuleId, netCode,   staCode, "", "",
                                moduleConfig,   keys.get()};

  // setup station configuration bindings
  auto &stationConfig{_stationConfigs[{netCode, staCode}]};

  std::string amplitudeProfileIdsStr;
  if (settings.getValue(amplitudeProfileIdsStr, "amplitudeProfiles") &&
      !amplitudeProfileIdsStr.empty()) {
    std::vector<std::string> amplitudeProfileIds;
    boost::algorithm::split(amplitudeProfileIds, amplitudeProfileIdsStr,
                            boost::is_any_of(settings::kConfigListSep));

    // parse amplitude processing configuration profiles
    for (const auto &amplitudeProfileId : amplitudeProfileIds) {
      std::string prefix{"amplitudes." + amplitudeProfileId};

      std::string locCode, chaCode;

      // the location code may be empty
      settings.getValue(locCode, prefix + ".locationCode");
      // a non-empty channel code is required to be specified
      if (!settings.getValue(chaCode, prefix + ".channelCode") ||
          chaCode.empty()) {
        continue;
      }

      auto &sensorLocationConfig{
          stationConfig._sensorLocationConfigs[{locCode, chaCode}]};
      auto &amplitudeProcessingConfig{
          sensorLocationConfig.amplitudeProcessingConfig};
      if (!settings.getValue(amplitudeProcessingConfig.enabled,
                             prefix + ".enable")) {
        amplitudeProcessingConfig.enabled =
            _defaultSensorLocationConfig.amplitudeProcessingConfig.enabled;
      }
      if (!settings.getValue(amplitudeProcessingConfig.filter,
                             prefix + ".filter")) {
        amplitudeProcessingConfig.filter =
            _defaultSensorLocationConfig.amplitudeProcessingConfig.filter;
      }
      if (!settings.getValue(amplitudeProcessingConfig.initTime,
                             prefix + ".initTime")) {
        amplitudeProcessingConfig.initTime =
            _defaultSensorLocationConfig.amplitudeProcessingConfig.initTime;
      }

      if (!amplitudeProcessingConfig.isValid()) {
        SCDETECT_LOG_WARNING(
            "Invalid configuration: failed to load amplitude profile with id: "
            "%s",
            amplitudeProfileId.c_str());
        continue;
      }
    }
  }
  stationConfig._parameters = keys;
  return stationConfig;
}

}  // namespace binding
}  // namespace detect
}  // namespace Seiscomp
