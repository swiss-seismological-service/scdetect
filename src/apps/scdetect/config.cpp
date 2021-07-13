#include "config.h"

#include <boost/property_tree/exceptions.hpp>
#include <utility>
#include <vector>

#include "exception.h"
#include "log.h"
#include "settings.h"
#include "utils.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {

namespace config {

BaseException::BaseException() : Exception("base config exception") {}

ParserException::ParserException()
    : BaseException{"error while parsing configuration"} {}

}  // namespace config

StreamConfig::StreamConfig() {}

StreamConfig::StreamConfig(const std::string &wfStreamId,
                           const std::string &filter, const double initTime,
                           const TemplateStreamConfig &templateConfig,
                           const std::string &templateId)
    : wfStreamId{wfStreamId},
      initTime{initTime},
      filter{filter},
      templateConfig{templateConfig} {}

StreamConfig::StreamConfig(const boost::property_tree::ptree &pt,
                           const StreamConfig &defaults)
    : templateId{pt.get<std::string>("templateId", utils::createUUID())

      },
      wfStreamId{pt.get<std::string>("waveformId")},
      initTime{pt.get<double>("initTime", defaults.initTime)},
      filter{pt.get_optional<std::string>("filter")},
      targetSamplingFrequency{
          pt.get_optional<double>("targetSamplingFrequency")} {
  templateConfig.phase =
      pt.get<std::string>("templatePhase", defaults.templateConfig.phase);
  templateConfig.wfStart =
      pt.get<double>("templateWaveformStart", defaults.templateConfig.wfStart);
  templateConfig.wfEnd =
      pt.get<double>("templateWaveformEnd", defaults.templateConfig.wfEnd);
  templateConfig.wfStreamId =
      pt.get<std::string>("templateWaveformId", wfStreamId);

  if (!filter && defaults.filter) {
    filter = defaults.filter;
  }

  templateConfig.filter = pt.get_optional<std::string>("templateFilter");
  if (!templateConfig.filter && defaults.templateConfig.filter) {
    templateConfig.filter = defaults.templateConfig.filter;
  }
}

bool StreamConfig::isValid() const {
  bool retval{true};
  try {
    retval = utils::WaveformStreamID{wfStreamId}.isValid();
  } catch (ValueException &e) {
    return false;
  }
  try {
    retval = utils::WaveformStreamID{templateConfig.wfStreamId}.isValid();
  } catch (ValueException &e) {
    return false;
  }

  const auto validateFilter = [](const std::string &filterId) {
    if (filterId.empty()) {
      return true;
    }
    std::string err;
    return config::validateFilter(filterId, err);
  };

  if (filter) {
    return validateFilter(*filter);
  }

  if (templateConfig.filter) {
    return validateFilter(*templateConfig.filter);
  }

  return (retval && templateConfig.wfStart < templateConfig.wfEnd &&
          !templateConfig.phase.empty() && utils::isGeZero(initTime));
}

bool DetectorConfig::isValid(size_t numStreamConfigs) const {
  return (config::validateXCorrThreshold(triggerOn) &&
          config::validateXCorrThreshold(triggerOff) &&
          (!gapInterpolation ||
           (gapInterpolation && utils::isGeZero(gapThreshold) &&
            utils::isGeZero(gapTolerance) && gapThreshold < gapTolerance)) &&
          config::validateArrivalOffsetThreshold(arrivalOffsetThreshold) &&
          config::validateMinArrivals(minArrivals,
                                      static_cast<int>(numStreamConfigs)));
}

TemplateConfig::TemplateConfig(const boost::property_tree::ptree &pt,
                               const DetectorConfig &detectorDefaults,
                               const StreamConfig &streamDefaults)
    : _detectorId{pt.get<std::string>("detectorId", utils::createUUID())},
      _originId(pt.get<std::string>("originId")) {
  _detectorConfig.triggerOn =
      pt.get<double>("triggerOnThreshold", detectorDefaults.triggerOn);
  _detectorConfig.triggerOff =
      pt.get<double>("triggerOffThreshold", detectorDefaults.triggerOff);
  _detectorConfig.triggerDuration =
      pt.get<double>("triggerDuration", detectorDefaults.triggerDuration);
  _detectorConfig.timeCorrection =
      pt.get<double>("timeCorrection", detectorDefaults.timeCorrection);
  _detectorConfig.gapInterpolation =
      pt.get<bool>("gapInterpolation", detectorDefaults.gapInterpolation);
  _detectorConfig.gapThreshold =
      pt.get<double>("gapThreshold", detectorDefaults.gapThreshold);
  _detectorConfig.gapTolerance =
      pt.get<double>("gapTolerance", detectorDefaults.gapTolerance);
  _detectorConfig.maximumLatency =
      pt.get<double>("maximumLatency", detectorDefaults.maximumLatency);
  _detectorConfig.createArrivals =
      pt.get<bool>("createArrivals", detectorDefaults.createArrivals);
  _detectorConfig.createTemplateArrivals = pt.get<bool>(
      "createTemplateArrivals", detectorDefaults.createTemplateArrivals);
  _detectorConfig.arrivalOffsetThreshold = pt.get<double>(
      "arrivalOffsetThreshold", detectorDefaults.arrivalOffsetThreshold);
  _detectorConfig.minArrivals =
      pt.get<int>("minimumArrivals", detectorDefaults.minArrivals);
  _detectorConfig.chunkSize =
      pt.get<double>("chunkSize", detectorDefaults.chunkSize);

  // patch stream defaults with detector config globals
  auto patchedStreamDefaults{streamDefaults};
  patchedStreamDefaults.initTime =
      pt.get<double>("initTime", streamDefaults.initTime);
  patchedStreamDefaults.filter = pt.get_optional<std::string>("filter");
  patchedStreamDefaults.targetSamplingFrequency =
      pt.get_optional<double>("targetSamplingFrequency");
  patchedStreamDefaults.templateConfig.phase =
      pt.get<std::string>("templatePhase", streamDefaults.templateConfig.phase);
  patchedStreamDefaults.templateConfig.wfStart = pt.get<double>(
      "templateWaveformStart", streamDefaults.templateConfig.wfStart);
  patchedStreamDefaults.templateConfig.wfEnd = pt.get<double>(
      "templateWaveformEnd", streamDefaults.templateConfig.wfEnd);
  patchedStreamDefaults.templateConfig.filter =
      pt.get_optional<std::string>("templateFilter");

  // initialize stream configs
  for (const auto &streamConfigPair : pt.find("streams")->second) {
    const auto &sc{streamConfigPair.second};

    std::string wfStreamId;
    try {
      StreamConfig streamConfig{sc, patchedStreamDefaults};
      _streamConfigs.emplace(streamConfig.wfStreamId, streamConfig);
      wfStreamId = streamConfig.wfStreamId;
    } catch (boost::property_tree::ptree_error &e) {
      throw config::ParserException{
          std::string{"Exception while parsing stream config: "} + e.what()};
    }

    if (!_streamConfigs[wfStreamId].isValid()) {
      throw config::ParserException{
          std::string{"Exception while parsing streamConfig: Invalid "
                      "stream configuration for stream: "} +
          wfStreamId};
    }
  }

  const auto maxArrivals{_streamConfigs.size()};
  if (_detectorConfig.minArrivals > static_cast<int>(maxArrivals)) {
    SCDETECT_LOG_WARNING_TAGGED(
        _detectorId,
        "Configured number of minimum arrivals exceeds number of configured "
        "streams (%d > %d). Resetting.",
        _detectorConfig.minArrivals, maxArrivals);
    _detectorConfig.minArrivals = maxArrivals;
  }

  if (!_detectorConfig.isValid(maxArrivals)) {
    throw config::ParserException{
        "Invalid template specific detector configuration"};
  }
}

const std::string TemplateConfig::detectorId() const { return _detectorId; }

const std::string TemplateConfig::originId() const { return _originId; }

const DetectorConfig TemplateConfig::detectorConfig() const {
  return _detectorConfig;
}

TemplateConfig::reference TemplateConfig::at(const std::string &stream_id) {
  return _streamConfigs.at(stream_id);
}

}  // namespace detect
}  // namespace Seiscomp
