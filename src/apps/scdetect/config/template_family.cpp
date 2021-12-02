#include "template_family.h"

#include "../util/waveform_stream_id.h"
#include "exception.h"

namespace Seiscomp {
namespace detect {
namespace config {

TemplateFamilyConfig::ReferenceConfig::TemplateConfigsIdx
    TemplateFamilyConfig::ReferenceConfig::_templateConfigsIdx;

const TemplateFamilyConfig::AllowedMagnitudeTypes
    TemplateFamilyConfig::_allowedMagnitudeTypes{"MLx"};

TemplateFamilyConfig::ReferenceConfig::ReferenceConfig(
    const boost::property_tree::ptree &pt,
    const std::vector<TemplateConfig> &templateConfigs,
    const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults) {
  // detector identifier
  const auto dId{pt.get_optional<std::string>("detectorId")};
  // origin identifier
  const auto oId{pt.get_optional<std::string>("originId")};
  if (dId && oId) {
    throw config::ParserException{
        "invalid configuration: both \"detectorId\" and \"originId\" "
        "specified"};
  }

  if (oId && !oId.value().empty()) {
    // explicit configuration
    for (const auto &streamConfigPair : pt.get_child("streams")) {
      const auto &streamConfigPt{streamConfigPair.second};
      SensorLocationConfig sensorLocationConfig;
      try {
        const auto waveformId{util::WaveformStreamID{
            streamConfigPt.get<std::string>("templateWaveformId")}};
        sensorLocationConfig.waveformId = waveformId.sensorLocationStreamId();
        sensorLocationConfig.channelId = waveformId.chaCode();
      } catch (ValueException &e) {
        throw config::ValidationError{"invalid configuration: " +
                                      std::string{e.what()}};
      }

      sensorLocationConfig.phase = streamConfigPt.get<std::string>(
          "templatePhase", sensorLocationDefaults.phase);
      if (sensorLocationConfig.phase.empty()) {
        sensorLocationConfig.phase = sensorLocationDefaults.phase;
      }
      sensorLocationConfig.waveformStart = streamConfigPt.get<double>(
          "templateWaveformStart", sensorLocationDefaults.waveformStart);
      sensorLocationConfig.waveformEnd = streamConfigPt.get<double>(
          "templateWaveformEnd", sensorLocationDefaults.waveformEnd);

      // XXX(damb): explicit references (i.e. those not referencing a detector
      // configuration) do not take limits into account.

      sensorLocationConfigs.emplace(sensorLocationConfig);
    }

    originId = *oId;
  } else if (dId && !dId.value().empty()) {
    if (!indexed()) {
      createIndex(templateConfigs);
    }

    // indirect configuration referencing a detector config
    const auto it{_templateConfigsIdx.find(*dId)};
    if (it == std::end(_templateConfigsIdx)) {
      throw config::ParserException{
          "invalid configuration: invalid \"detectorId\": " + *detectorId};
    }

    for (const auto &streamConfigPair : pt.get_child("streams")) {
      const auto &streamConfigPt{streamConfigPair.second};

      SensorLocationConfig sensorLocationConfig;
      detect::config::StreamConfig detectorStreamConfig;
      try {
        const auto waveformId{util::WaveformStreamID{
            streamConfigPt.get<std::string>("templateWaveformId")}};

        detectorStreamConfig = it->second->at(util::to_string(waveformId));

        sensorLocationConfig.waveformId = waveformId.sensorLocationStreamId();
        sensorLocationConfig.channelId = waveformId.chaCode();
        sensorLocationConfig.lowerLimit =
            streamConfigPt.get_optional<double>("lowerLimit");
        if (!sensorLocationConfig.lowerLimit) {
          sensorLocationConfig.lowerLimit = sensorLocationDefaults.lowerLimit;
        }
        sensorLocationConfig.upperLimit =
            streamConfigPt.get_optional<double>("upperLimit");
        if (!sensorLocationConfig.upperLimit) {
          sensorLocationConfig.upperLimit = sensorLocationDefaults.upperLimit;
        }
        if (sensorLocationConfig.lowerLimit &&
            sensorLocationConfig.upperLimit &&
            (*sensorLocationConfig.upperLimit <=
             *sensorLocationConfig.lowerLimit)) {
          throw config::ValidationError{
              "invalid configuration: \"upperLimit\" must be greater than "
              "\"lowerLimit\""};
        }

      } catch (std::out_of_range &e) {
        throw config::ParserException{
            "invalid configuration: failed to look up stream configuration for "
            "stream: " +
            streamConfigPt.get<std::string>("templateWaveformId")};
      } catch (ValueException &e) {
        throw config::ValidationError{"invalid configuration: " +
                                      std::string{e.what()}};
      }

      sensorLocationConfig.phase = detectorStreamConfig.templateConfig.phase;
      sensorLocationConfig.waveformStart =
          detectorStreamConfig.templateConfig.wfStart;
      sensorLocationConfig.waveformEnd =
          detectorStreamConfig.templateConfig.wfEnd;

      sensorLocationConfigs.emplace(sensorLocationConfig);
    }

    if (sensorLocationConfigs.empty()) {
      throw config::ParserException{
          "invalid configuration: no stream configuration found"};
    }

    originId = it->second->originId();
    detectorId = *dId;
  } else {
    throw config::ParserException{
        "invalid configuration: neither \"detectorId\" nor \"originId\" "
        "specified"};
  }
}

bool TemplateFamilyConfig::ReferenceConfig::referencesDetector() const {
  return static_cast<bool>(detectorId);
}

void TemplateFamilyConfig::ReferenceConfig::createIndex(
    const TemplateFamilyConfig::ReferenceConfig::TemplateConfigs
        &templateConfigs) {
  for (auto it{std::begin(templateConfigs)}; it != std::end(templateConfigs);
       ++it) {
    _templateConfigsIdx.emplace(it->detectorId(), it);
  }
}

bool TemplateFamilyConfig::ReferenceConfig::indexed() const {
  return !_templateConfigsIdx.empty();
}

TemplateFamilyConfig::TemplateFamilyConfig(
    const boost::property_tree::ptree &pt,
    const std::vector<TemplateConfig> &templateConfigs,
    const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults)
    : _id{pt.get<std::string>("id", util::createUUID())},
      _magnitudeType{pt.get<std::string>("magnitudeType", "MLx")} {
  validateMagnitudeType(_magnitudeType);

  // parse template family configuration defaults
  const auto lowerLimitDefault{pt.get_optional<double>("lowerLimit")};
  const auto upperLimitDefault{pt.get_optional<double>("upperLimit")};
  if (lowerLimitDefault && upperLimitDefault &&
      *upperLimitDefault <= *lowerLimitDefault) {
    throw config::ValidationError{
        "invalid configuration: `\"lowerLimit\" must be greater than "
        "\"upperLimit\""};
  }
  ReferenceConfig::SensorLocationConfig sensorLocationDefaultsWithGlobals{
      sensorLocationDefaults};
  sensorLocationDefaultsWithGlobals.lowerLimit = lowerLimitDefault;
  sensorLocationDefaultsWithGlobals.upperLimit = upperLimitDefault;

  loadReferenceConfigs(pt.get_child("references"), templateConfigs,
                       sensorLocationDefaultsWithGlobals);
}

const std::string &TemplateFamilyConfig::id() const { return _id; }

const std::string &TemplateFamilyConfig::magnitudeType() const {
  return _magnitudeType;
}

void TemplateFamilyConfig::loadReferenceConfigs(
    const boost::property_tree::ptree &pt,
    const std::vector<TemplateConfig> &templateConfigs,
    const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults) {
  bool hasDetectorReference{false};
  for (const auto &referenceConfigPair : pt) {
    const auto &referenceConfigPt{referenceConfigPair.second};

    ReferenceConfig referenceConfig{referenceConfigPt, templateConfigs,
                                    sensorLocationDefaults};
    if (referenceConfig.referencesDetector()) {
      hasDetectorReference = true;
    }
    _referenceConfigs.emplace(referenceConfig);
  }

  if (!hasDetectorReference) {
    throw config::ValidationError{
        "missing detector reference: template families must at least define a "
        "single reference to a detector"};
  }
}

void TemplateFamilyConfig::validateMagnitudeType(
    const std::string &magnitudeType) {
  if (_allowedMagnitudeTypes.find(magnitudeType) ==
      std::end(_allowedMagnitudeTypes)) {
    throw config::ValidationError{
        "invalid configuration: invalid magnitude type: " + magnitudeType};
  }
}

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp
