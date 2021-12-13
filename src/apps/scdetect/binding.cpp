#include "binding.h"

#include <seiscomp/datamodel/configstation.h>
#include <seiscomp/datamodel/parameter.h>
#include <seiscomp/datamodel/parameterset.h>
#include <seiscomp/datamodel/setup.h>
#include <seiscomp/datamodel/utils.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <string>
#include <vector>

#include "config/validators.h"
#include "exception.h"
#include "log.h"
#include "seiscomp/core/strings.h"
#include "settings.h"
#include "util/memory.h"
#include "util/util.h"

namespace Seiscomp {
namespace detect {
namespace binding {

StreamConfig::DeconvolutionConfig::operator AmplitudeProcessor::
    DeconvolutionConfig() const {
  AmplitudeProcessor::DeconvolutionConfig retval;
  retval.enabled = enabled;
  retval.responseTaperLength = responseTaperLength;
  retval.minimumResponseTaperFrequency = minimumResponseTaperFrequency;
  retval.maximumResponseTaperFrequency = maximumResponseTaperFrequency;
  return retval;
}

std::string StreamConfig::DeconvolutionConfig::debugString() const {
  return "enabled: " + std::to_string(enabled) +
         ", responseTaperLength: " + std::to_string(responseTaperLength) +
         "s, minimumResponseTaperFrequency: " +
         std::to_string(minimumResponseTaperFrequency) +
         "Hz, maximumResponseTaperFrequency: " +
         std::to_string(maximumResponseTaperFrequency) + "Hz";
}

const StreamConfig &SensorLocationConfig::at(const std::string &chaCode) const {
  return streamConfigs.at(chaCode);
}

StationConfig::const_iterator StationConfig::begin() const {
  return _sensorLocationConfigs.begin();
}

StationConfig::const_iterator StationConfig::end() const {
  return _sensorLocationConfigs.end();
}

const SensorLocationConfig &StationConfig::at(
    const std::string &locCode, const std::string &chaCode) const {
  return _sensorLocationConfigs.at({locCode, getBandAndSourceCode(chaCode)});
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
  return _stationConfigs.at({netCode, staCode})
      .at(locCode, getBandAndSourceCode(chaCode));
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
    keys = util::make_smart<Util::KeyValues>();
    keys->init(parameterSet);
  }

  // XXX(damb): merges module configuration with bindings
  Processing::Settings settings{configModuleId, netCode,   staCode, "", "",
                                moduleConfig,   keys.get()};

  // setup station configuration bindings
  auto &stationConfig{_stationConfigs[{netCode, staCode}]};

  std::string sensorLocationProfileIdsStr;
  if (settings.getValue(sensorLocationProfileIdsStr,
                        "sensorLocationProfiles") &&
      !sensorLocationProfileIdsStr.empty()) {
    std::vector<std::string> sensorLocationProfileIds;
    boost::algorithm::split(sensorLocationProfileIds,
                            sensorLocationProfileIdsStr,
                            boost::is_any_of(settings::kConfigListSep));

    // parse sensor location profiles
    for (const auto &sensorLocationProfileId : sensorLocationProfileIds) {
      std::string prefix{"sensorLocation." + sensorLocationProfileId};

      std::string locCode, chaCode;

      // the location code may be empty
      settings.getValue(locCode, prefix + ".locationCode");
      // a non-empty channel code is required to be specified
      if (!settings.getValue(chaCode, prefix + ".channelCode") ||
          chaCode.empty() || chaCode.size() < 2) {
        continue;
      }

      // only take the band and source code identifiers into account
      auto &sensorLocationConfig{stationConfig._sensorLocationConfigs[{
          locCode, getBandAndSourceCode(chaCode)}]};
      // load amplitude processing config
      try {
        auto amplitudePrefix{prefix + ".amplitudes"};
        detail::load(settings, amplitudePrefix,
                     sensorLocationConfig.amplitudeProcessingConfig,
                     _defaultSensorLocationConfig.amplitudeProcessingConfig);

        std::string responseProfileIdsStr;
        if (settings.getValue(
                responseProfileIdsStr,
                amplitudePrefix + ".responses.responseProfiles") &&
            !responseProfileIdsStr.empty()) {
          std::vector<std::string> responseProfileIds;
          boost::algorithm::split(responseProfileIds, responseProfileIdsStr,
                                  boost::is_any_of(settings::kConfigListSep));

          // parse response configuration profiles
          for (const auto &responseProfileId : responseProfileIds) {
            std::string respProfilePrefix{amplitudePrefix + ".responses." +
                                          responseProfileId};

            std::string subSourceCode;
            // a non-empty subSourceCode is required to be specified
            if (!settings.getValue(subSourceCode,
                                   respProfilePrefix + ".channelCode") ||
                subSourceCode.empty() || subSourceCode.size() != 1) {
              continue;
            }

            auto &streamConfig{
                sensorLocationConfig
                    .streamConfigs[getBandAndSourceCode(chaCode) +
                                   subSourceCode]};
            try {
              detail::load(settings, respProfilePrefix, streamConfig,
                           _defaultSensorLocationConfig._defaultStreamConfig);
            } catch (ValueException &e) {
              throw ValueException{"failed to load response profile with id: " +
                                   responseProfileId + " (reason: " + e.what() +
                                   ")"};
            }
          }
        }
      } catch (ValueException &e) {
        SCDETECT_LOG_WARNING(
            "Invalid amplitude processing configuration: failed to load "
            "amplitude processing configuration for sensor location profile "
            "with id: %s (reason: %s)",
            sensorLocationProfileId.c_str(), e.what());
        continue;
      }

      // load magnitude processing config
      try {
        auto magnitudePrefix{prefix + ".magnitudes"};
        detail::load(settings, magnitudePrefix,
                     sensorLocationConfig.magnitudeProcessingConfig,
                     _defaultSensorLocationConfig.magnitudeProcessingConfig);
      } catch (ValueException &e) {
        SCDETECT_LOG_WARNING(
            "Invalid magnitude processing configuration: failed to load "
            "magnitude processing configuration for sensor location profile "
            "with id: %s (reason: %s)",
            sensorLocationProfileId.c_str(), e.what());
        continue;
      }

      detail::validate(sensorLocationConfig);
    }
  }
  stationConfig._parameters = keys;
  return stationConfig;
}

std::string getBandAndSourceCode(const std::string &chaCode) {
  return chaCode.size() <= 2 ? chaCode : chaCode.substr(0, 2);
}

boost::optional<double> parseSaturationThreshold(
    const Processing::Settings &settings, const std::string &parameter) {
  std::string saturationThresholdStr;
  if (!settings.getValue(saturationThresholdStr, parameter)) {
    return boost::none;
  }

  boost::trim(saturationThresholdStr);

  if (saturationThresholdStr == "false") {
    return boost::none;
  }

  size_t atPosition{saturationThresholdStr.find('@')};
  double saturationThreshold{0};
  if (atPosition == std::string::npos) {
    // value given as absolute value
    if (!Core::fromString(saturationThreshold, saturationThresholdStr)) {
      throw ValueException{"invalid saturation threshold: " +
                           saturationThresholdStr};
    }
    return saturationThreshold;

  } else {
    std::string bitsStr{saturationThresholdStr.substr(atPosition + 1)};
    int bits;
    double value;

    if (bitsStr.empty()) {
      throw ValueException{
          "invalid saturation threshold: no effective bits specified: " +
          saturationThresholdStr};
    }

    if (!Core::fromString(bits, bitsStr)) {
      throw ValueException{
          "invalid saturation threshold: invalid saturation threshold bits "
          "specified: " +
          saturationThresholdStr};
    }

    if (bits <= 0 || bits > 64) {
      throw ValueException{
          "invalid saturation threshold: number of effective bits out of "
          "range: " +
          saturationThresholdStr};
    }

    std::string valueStr{saturationThresholdStr.substr(0, atPosition)};
    boost::trim(valueStr);

    if (valueStr.empty()) {
      throw ValueException{
          "invalid saturation threshold: saturation threshold relative value "
          "is empty: " +
          saturationThresholdStr};
    }

    bool isPercent{false};
    if (*valueStr.rbegin() == '%') {
      isPercent = true;
      valueStr.resize(valueStr.size() - 1);
    }

    if (valueStr.empty()) {
      throw ValueException{
          "invalid saturation threshold: saturation threshold relative value "
          "is empty: " +
          saturationThresholdStr};
    }

    if (!Core::fromString(value, valueStr)) {
      throw ValueException{
          "invalid saturation threshold: invalid saturation threshold relative "
          "value: " +
          saturationThresholdStr};
    }

    if (isPercent) value *= 0.01;

    if (value < 0 || value > 1) {
      throw ValueException{
          "invalid saturation threshold: number of relative value out of range "
          "[0,1]: " +
          saturationThresholdStr};
    }

    return (1 << bits) * value;
  }
  return boost::none;
}

namespace detail {

void setFilter(const std::string &filter, std::string &storageLocation) {
  // filtering explictily disabled
  if (filter.empty()) {
    storageLocation = filter;
    return;
  }

  std::string err;
  if (!config::validateFilter(filter, err)) {
    throw ValueException{"invalid filter string identifier: " + err};
  }
  storageLocation = filter;
}

void setInitTime(double t, double &storageLocation) {
  if (!util::isGeZero(t)) {
    throw ValueException{"invalid init time: " + std::to_string(t) +
                         " (Must be >= 0.)"};
  }
  storageLocation = t;
}

void setFilter(const Processing::Settings &settings,
               const std::string &parameterFilter,
               const std::string &parameterInitTime,
               const std::string &defaultFilter, double defaultInitTime,
               std::string &storageLocationFilter,
               double &storageLocationInitTime) {
  std::string filterStr;
  if (!settings.getValue(filterStr, parameterFilter)) {
    filterStr = defaultFilter;
  }

  double t;
  if (!settings.getValue(t, parameterInitTime)) {
    t = defaultInitTime;
  }

  std::string previousFilter{storageLocationFilter};
  setFilter(filterStr, storageLocationFilter);
  try {
    setInitTime(t, storageLocationInitTime);
  } catch (ValueException &) {
    storageLocationFilter = previousFilter;
    throw;
  }
}

void setSaturationThreshold(const Processing::Settings &settings,
                            const std::string &parameter,
                            boost::optional<double> &storageLocation) {
  storageLocation = parseSaturationThreshold(settings, parameter);
}

void setResponseTaperLength(double length, double &storageLocation) {
  if (!util::isGeZero(length)) {
    throw ValueException{"invalid response taper length: " +
                         std::to_string(length) + " (Must be >= 0.)"};
  }
  storageLocation = length;
}

void setResponseTaperLength(const Processing::Settings &settings,
                            const std::string &parameter,
                            double &storageLocation,
                            const boost::optional<double> &defaultValue) {
  double length;
  if (!settings.getValue(length, parameter)) {
    if (!defaultValue) {
      throw ValueException{"failed to load response taper length"};
    }
    length = *defaultValue;
  }
  setResponseTaperLength(length, storageLocation);
}

void setResponseTaperFrequency(double f, double &storageLocation) {
  // XXX(damb): values <= 0 are valid, too
  storageLocation = f;
}

void setResponseTaperFrequency(const Processing::Settings &settings,
                               const std::string &parameter,
                               double &storageLocation,
                               const boost::optional<double> &defaultValue) {
  double f;
  if (!settings.getValue(f, parameter)) {
    if (!defaultValue) {
      throw ValueException{"failed to load response taper frequency"};
    }
    f = *defaultValue;
  }
  setResponseTaperFrequency(f, storageLocation);
}

void load(const Processing::Settings &settings,
          const std::string &parameterPrefix, StreamConfig &storageLocation,
          const StreamConfig &defaults) {
  setResponseTaperLength(
      settings, parameterPrefix + ".taperLength",
      storageLocation.deconvolutionConfig.responseTaperLength,
      defaults.deconvolutionConfig.responseTaperLength);
  setResponseTaperFrequency(
      settings, parameterPrefix + ".minimumTaperFrequency",
      storageLocation.deconvolutionConfig.minimumResponseTaperFrequency,
      defaults.deconvolutionConfig.minimumResponseTaperFrequency);
  setResponseTaperFrequency(
      settings, parameterPrefix + ".maximumTaperFrequency",
      storageLocation.deconvolutionConfig.maximumResponseTaperFrequency,
      defaults.deconvolutionConfig.maximumResponseTaperFrequency);
}

void load(const Processing::Settings &settings,
          const std::string &parameterPrefix,
          SensorLocationConfig::AmplitudeProcessingConfig &storageLocation,
          const SensorLocationConfig::AmplitudeProcessingConfig &defaults) {
  std::string amplitudeTypes;
  if (!settings.getValue(amplitudeTypes, parameterPrefix + ".amplitudes") ||
      amplitudeTypes.empty()) {
    storageLocation.amplitudeTypes = defaults.amplitudeTypes;
  } else {
    std::vector<std::string> tokens;
    Core::split(tokens, amplitudeTypes, settings::kConfigListSep.c_str());
    for (const auto &t : tokens) {
      if (config::validateAmplitudeType(t)) {
        storageLocation.amplitudeTypes.push_back(t);
      }
    }
  }

  if (!settings.getValue(storageLocation.enabled,
                         parameterPrefix + ".enable")) {
    storageLocation.enabled = defaults.enabled;
  }

  setFilter(settings, parameterPrefix + ".filter",
            parameterPrefix + ".initTime", defaults.mlx.filter,
            defaults.mlx.initTime, storageLocation.mlx.filter,
            storageLocation.mlx.initTime);

  setSaturationThreshold(settings, parameterPrefix + ".saturationThreshold",
                         storageLocation.mlx.saturationThreshold);
}

void load(const Processing::Settings &settings,
          const std::string &parameterPrefix,
          SensorLocationConfig::MagnitudeProcessingConfig &storageLocation,
          const SensorLocationConfig::MagnitudeProcessingConfig &defaults) {
  std::string magnitudeTypes;
  if (!settings.getValue(magnitudeTypes, parameterPrefix + ".magnitudes") ||
      magnitudeTypes.empty()) {
    storageLocation.magnitudeTypes = defaults.magnitudeTypes;
  } else {
    std::vector<std::string> tokens;
    Core::split(tokens, magnitudeTypes, settings::kConfigListSep.c_str());
    for (const auto &t : tokens) {
      if (config::validateMagnitudeType(t)) {
        storageLocation.magnitudeTypes.push_back(t);
      }
    }
  }

  if (!settings.getValue(storageLocation.enabled,
                         parameterPrefix + ".enable")) {
    storageLocation.enabled = defaults.enabled;
  }
}

void validate(SensorLocationConfig &config) {
  if (!config.amplitudeProcessingConfig.enabled) {
    config.magnitudeProcessingConfig.enabled = false;
  } else {
    // compare magnitude types with amplitude types
    std::vector<std::string> compared;
    for (const auto &magnitudeType :
         config.magnitudeProcessingConfig.magnitudeTypes) {
      const auto &amplitudeTypes{
          config.amplitudeProcessingConfig.amplitudeTypes};
      bool amplitudeTypeForMagnitudeTypeIsEnabled{
          std::find(std::begin(amplitudeTypes), std::end(amplitudeTypes),
                    magnitudeType) != std::end(amplitudeTypes)};
      if (amplitudeTypeForMagnitudeTypeIsEnabled) {
        compared.push_back(magnitudeType);
      }

      config.magnitudeProcessingConfig.magnitudeTypes = compared;
    }
  }
}

}  // namespace detail
}  // namespace binding
}  // namespace detect
}  // namespace Seiscomp
