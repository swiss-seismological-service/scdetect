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

#include "exception.h"
#include "log.h"
#include "settings.h"
#include "util/memory.h"
#include "util/util.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {
namespace binding {

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

StreamConfig::DeconvolutionConfig::operator AmplitudeProcessor::
    DeconvolutionConfig() const {
  AmplitudeProcessor::DeconvolutionConfig retval;
  retval.enabled = enabled;
  retval.responseTaperLength = responseTaperLength;
  retval.minimumResponseTaperFrequency = minimumResponseTaperFrequency;
  retval.maximumResponseTaperFrequency = maximumResponseTaperFrequency;
  return retval;
}

void StreamConfig::DeconvolutionConfig::setResponseTaperLength(double length) {
  if (!util::isGeZero(length)) {
    throw ValueException{"invalid response taper length: " +
                         std::to_string(length) + " (Must be >= 0.)"};
  }
  responseTaperLength = length;
}

void StreamConfig::DeconvolutionConfig::setResponseTaperLength(
    const Processing::Settings &settings, const std::string &parameter,
    const StreamConfig::DeconvolutionConfig &defaultConfig) {
  double length;
  if (!settings.getValue(length, parameter)) {
    length = defaultConfig.responseTaperLength;
  }
  setResponseTaperLength(length);
}

void StreamConfig::DeconvolutionConfig::setMinimumResponseTaperFrequency(
    double f) {
  minimumResponseTaperFrequency = f;
}

void StreamConfig::DeconvolutionConfig::setMinimumResponseTaperFrequency(
    const Processing::Settings &settings, const std::string &parameter,
    const DeconvolutionConfig &defaultConfig) {
  double f;
  if (!settings.getValue(f, parameter)) {
    f = defaultConfig.minimumResponseTaperFrequency;
  }
  setMinimumResponseTaperFrequency(f);
}

void StreamConfig::DeconvolutionConfig::setMaximumResponseTaperFrequency(
    double f) {
  maximumResponseTaperFrequency = f;
}

void StreamConfig::DeconvolutionConfig::setMaximumResponseTaperFrequency(
    const Processing::Settings &settings, const std::string &parameter,
    const DeconvolutionConfig &defaultConfig) {
  double f;
  if (!settings.getValue(f, parameter)) {
    f = defaultConfig.maximumResponseTaperFrequency;
  }
  setMaximumResponseTaperFrequency(f);
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

void SensorLocationConfig::AmplitudeProcessingConfig::setFilter(
    const std::string &filterStr) {
  if (filterStr.empty()) {
    filter = filterStr;
    return;
  }

  std::string err;
  if (!config::validateFilter(filterStr, err)) {
    throw ValueException{"invalid filter string identifier: " + err};
  }
  filter = filterStr;
}

void SensorLocationConfig::AmplitudeProcessingConfig::setInitTime(double t) {
  if (!util::isGeZero(t)) {
    throw ValueException{"invalid init time: " + std::to_string(t) +
                         " (Must be >= 0.)"};
  }
  initTime = t;
}

void SensorLocationConfig::AmplitudeProcessingConfig::setFilter(
    const Processing::Settings &settings, const std::string &parameterFilter,
    const std::string &parameterInitTime,
    const SensorLocationConfig::AmplitudeProcessingConfig &defaultConfig) {
  std::string filterStr;
  if (!settings.getValue(filterStr, parameterFilter)) {
    filterStr = defaultConfig.filter;
  }

  double t;
  if (!settings.getValue(t, parameterInitTime)) {
    t = defaultConfig.initTime;
  }

  std::string previousFilter{filter};
  setFilter(filterStr);
  try {
    setInitTime(t);
  } catch (ValueException &) {
    filter = previousFilter;
    throw;
  }
}

void SensorLocationConfig::AmplitudeProcessingConfig::setSaturationThreshold(
    const Processing::Settings &settings, const std::string &parameter) {
  saturationThreshold = parseSaturationThreshold(settings, parameter);
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
    keys = util::make_smart<Util::KeyValues>();
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
          chaCode.empty() || chaCode.size() < 2) {
        continue;
      }

      // only take the band and source code identifiers into account
      chaCode = chaCode.substr(0, 2);
      auto &sensorLocationConfig{
          stationConfig._sensorLocationConfigs[{locCode, chaCode}]};
      auto &amplitudeProcessingConfig{
          sensorLocationConfig.amplitudeProcessingConfig};
      if (!settings.getValue(amplitudeProcessingConfig.enabled,
                             prefix + ".enable")) {
        amplitudeProcessingConfig.enabled =
            _defaultSensorLocationConfig.amplitudeProcessingConfig.enabled;
      }

      try {
        amplitudeProcessingConfig.setFilter(
            settings, prefix + ".filter", prefix + ".initTime",
            _defaultSensorLocationConfig.amplitudeProcessingConfig);

        amplitudeProcessingConfig.setSaturationThreshold(
            settings, prefix + ".saturationThreshold");
      } catch (ValueException &e) {
        SCDETECT_LOG_WARNING(
            "Invalid configuration: failed to load amplitude profile with "
            "id: %s (reason: %s)",
            amplitudeProfileId.c_str(), e.what());
        continue;
      }

      std::string responseProfileIdsStr;
      if (settings.getValue(responseProfileIdsStr,
                            prefix + ".responses.responseProfiles") &&
          !responseProfileIdsStr.empty()) {
        std::vector<std::string> responseProfileIds;
        boost::algorithm::split(responseProfileIds, responseProfileIdsStr,
                                boost::is_any_of(settings::kConfigListSep));

        // parse response configuration profiles
        for (const auto &responseProfileId : responseProfileIds) {
          std::string respProfilePrefix{prefix + ".responses." +
                                        responseProfileId};

          std::string subSourceCode;
          // a non-empty subSourceCode is required to be specified
          if (!settings.getValue(subSourceCode,
                                 respProfilePrefix + ".channelCode") ||
              subSourceCode.empty() || subSourceCode.size() != 1) {
            continue;
          }

          auto &streamConfig{
              sensorLocationConfig.streamConfigs[chaCode + subSourceCode]};
          auto &deconvolutionConfig{streamConfig.deconvolutionConfig};
          try {
            deconvolutionConfig.setResponseTaperLength(
                settings, respProfilePrefix + ".taperLength",
                deconvolutionConfig);
            deconvolutionConfig.setMinimumResponseTaperFrequency(
                settings, respProfilePrefix + ".minimumTaperFrequency",
                deconvolutionConfig);
            deconvolutionConfig.setMaximumResponseTaperFrequency(
                settings, respProfilePrefix + ".maximumTaperFrequency",
                deconvolutionConfig);
          } catch (ValueException &e) {
            SCDETECT_LOG_WARNING(
                "Invalid configuration: failed to load response profile with "
                "id: %s (reason: %s)",
                responseProfileId.c_str(), e.what());
            continue;
          }
        }
      }
    }
  }
  stationConfig._parameters = keys;
  return stationConfig;
}

}  // namespace binding
}  // namespace detect
}  // namespace Seiscomp
