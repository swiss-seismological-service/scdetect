#include "app.h"

#include <seiscomp/core/arrayfactory.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/strings.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/comment.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/notifier.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/phase.h>
#include <seiscomp/datamodel/realquantity.h>
#include <seiscomp/datamodel/timequantity.h>
#include <seiscomp/datamodel/timewindow.h>
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/io/recordinput.h>
#include <seiscomp/math/geo.h>
#include <seiscomp/processing/stream.h>
#include <seiscomp/utils/files.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/smart_ptr/intrusive_ptr.hpp>
#include <exception>
#include <ios>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "amplitude/factory.h"
#include "amplitude_processor.h"
#include "config/exception.h"
#include "config/validators.h"
#include "detector/arrival.h"
#include "detector/detectorbuilder.h"
#include "detector/detectorwaveformprocessor.h"
#include "eventstore.h"
#include "log.h"
#include "magnitude/regression.h"
#include "magnitude_processor.h"
#include "processing/processor.h"
#include "processing/waveform_processor.h"
#include "resamplerstore.h"
#include "settings.h"
#include "util/horizontal_components.h"
#include "util/memory.h"
#include "util/util.h"
#include "util/waveform_stream_id.h"
#include "version.h"

namespace Seiscomp {
namespace detect {

Application::Application(int argc, char **argv)
    : StreamApplication(argc, argv) {
  setLoadStationsEnabled(true);
  setLoadInventoryEnabled(true);
  setLoadConfigModuleEnabled(true);
  setMessagingEnabled(true);

  setPrimaryMessagingGroup("LOCATION");
}

Application::BaseException::BaseException()
    : Exception{"base application exception"} {}

Application::ConfigError::ConfigError()
    : BaseException{"application configuration error"} {}

Application::DuplicatePublicObjectId::DuplicatePublicObjectId()
    : BaseException{"duplicate public object identifier"} {}

const char *Application::version() { return kVersion; }

void Application::createCommandLineDescription() {
  StreamApplication::createCommandLineDescription();

  commandline().addOption(
      "Database", "event-db",
      "load events from the given database or file, format: "
      "[service://]location",
      &_config.urlEventDb);

  commandline().addOption(
      "Records", "record-starttime",
      "defines a start time (YYYY-MM-DDTHH:MM:SS formatted) for "
      "requesting records from the configured archive recordstream; "
      "implicitly enables reprocessing/playback mode",
      &_config.playbackConfig.startTimeStr);
  commandline().addOption(
      "Records", "record-endtime",
      "defines an end time (YYYY-MM-DDTHH:MM:SS formatted) for "
      "requesting records from the configured archive recordstream; "
      "implicitly enables reprocessing/playback mode",
      &_config.playbackConfig.endTimeStr);

  commandline().addGroup("Mode");
  commandline().addOption(
      "Mode", "offline",
      "offline mode, do not connect to the messaging system (implies "
      "--no-publish i.e. no objects are sent)");
  commandline().addOption("Mode", "no-publish", "do not send any objects");
  commandline().addOption(
      "Mode", "ep",
      "same as --no-publish, but outputs all event parameters scml "
      "formatted; specifying the output path as '-' (a single dash) will "
      "force the output to be redirected to stdout",
      &_config.pathEp);
  commandline().addOption(
      "Mode", "playback",
      "Use playback mode that does not restrict the maximum allowed "
      "data latency");
  commandline().addOption(
      "Mode", "templates-prepare",
      "load template waveform data from the configured recordstream "
      "and save it in the module's caching directory, then exit");
  commandline().addOption(
      "Mode", "templates-reload",
      "force reloading template waveform data and omit cached waveform data");
  commandline().addOption(
      "Mode", "amplitudes-force",
      "enables/disables the calculation of amplitudes regardless of the "
      "configuration provided on detector configuration level granularity",
      &_config.amplitudesForceMode, false);
  commandline().addOption(
      "Mode", "magnitudes-force",
      "enables/disables the calculation of magnitudes regardless of the "
      "configuration provided on detector configuration level granularity",
      &_config.magnitudesForceMode, false);

  commandline().addGroup("Input");
  commandline().addOption(
      "Input", "templates-json",
      "path to a template configuration file (json-formatted)",
      &_config.pathTemplateJson);
  commandline().addOption(
      "Input", "templates-family-json",
      "path to a template family configuration file (json-formatted)",
      &_config.pathTemplateFamilyJson);
}

bool Application::validateParameters() {
  if (!StreamApplication::validateParameters()) return false;

  // validate paths
  if (!_config.pathTemplateJson.empty() &&
      !Util::fileExists(_config.pathTemplateJson)) {
    SCDETECT_LOG_ERROR("Invalid path to template configuration file: %s",
                       _config.pathTemplateJson.c_str());
    return false;
  }
  if (!_config.pathTemplateFamilyJson.empty() &&
      !Util::fileExists(_config.pathTemplateFamilyJson)) {
    SCDETECT_LOG_ERROR("Invalid path to template family configuration file: %s",
                       _config.pathTemplateFamilyJson.c_str());
    return false;
  }

  // validate reprocessing config
  auto validateAndStoreTime = [](const std::string &timeStr,
                                 Core::Time &result) {
    if (!timeStr.empty() && !result.fromString(timeStr.c_str(), "%FT%T")) {
      SCDETECT_LOG_ERROR("Invalid time: %s", timeStr.c_str());
      return false;
    }
    return true;
  };

  if (!validateAndStoreTime(_config.playbackConfig.startTimeStr,
                            _config.playbackConfig.startTime)) {
    return false;
  }
  if (!validateAndStoreTime(_config.playbackConfig.endTimeStr,
                            _config.playbackConfig.endTime)) {
    return false;
  }

  if (!config::validateXCorrThreshold(_config.detectorConfig.triggerOn)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'triggerOnThreshold': %f. Not in "
        "interval [-1,1].",
        _config.detectorConfig.triggerOn);
    return false;
  }
  if (!config::validateXCorrThreshold(_config.detectorConfig.triggerOff)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'triggerOffThreshold': %f. Not in "
        "interval [-1,1].",
        _config.detectorConfig.triggerOff);
    return false;
  }

  if (_config.streamConfig.filter && !(*_config.streamConfig.filter).empty()) {
    std::string err;
    if (!config::validateFilter(*_config.streamConfig.filter, err)) {
      SCDETECT_LOG_ERROR("Invalid configuration: 'filter': %s (%s)",
                         (*_config.streamConfig.filter).c_str(), err.c_str());
      return false;
    }
  }
  if (!util::isGeZero(_config.streamConfig.initTime)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'initTime': %f. Must be "
        "greater equal 0.",
        _config.streamConfig.initTime);
    return false;
  }
  if (!config::validateArrivalOffsetThreshold(
          _config.detectorConfig.arrivalOffsetThreshold)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'arrivalOffsetThreshold': %f. "
        "Must be < 0 or >= 2.0e-6",
        _config.detectorConfig.arrivalOffsetThreshold);
    return false;
  }
  if (!config::validateMinArrivals(_config.detectorConfig.minArrivals)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'minimumArrivals': %d. "
        "Must be < 0 or >= 1",
        _config.detectorConfig.minArrivals);
    return false;
  }
  if (!config::validateLinkerMergingStrategy(
          _config.detectorConfig.mergingStrategy)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'mergingStrategy': %s. Must be one of: {%s}",
        _config.detectorConfig.mergingStrategy.c_str(),
        boost::algorithm::join(config::kValidLinkerMergingStrategies, ",")
            .c_str());

    return false;
  }
  if (_config.streamConfig.templateConfig.wfStart >=
      _config.streamConfig.templateConfig.wfEnd) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'waveformStart' >= 'waveformEnd' : %f >= %f",
        _config.streamConfig.templateConfig.wfStart,
        _config.streamConfig.templateConfig.wfEnd);
    return false;
  }

  return true;
}

bool Application::handleCommandLineOptions() {
  _config.init(commandline());

  if (_config.offlineMode) {
    SCDETECT_LOG_INFO("Disable messaging");
    setMessagingEnabled(false);

    _config.noPublish = true;
  }

  if (!_config.noPublish && commandline().hasOption("ep")) {
    _config.noPublish = true;
  }

  bool magnitudesForcedEnabled{_config.magnitudesForceMode &&
                               *_config.magnitudesForceMode};
  bool amplitudesForcedDisabled{_config.amplitudesForceMode &&
                                !*_config.amplitudesForceMode &&
                                !magnitudesForcedEnabled};
  // disable the database if required
  if (!isInventoryDatabaseEnabled() && !isEventDatabaseEnabled() &&
      (amplitudesForcedDisabled || !isConfigDatabaseEnabled())) {
    SCDETECT_LOG_INFO("Disable database connection");
    setDatabaseEnabled(false, false);
  }

  return false;
}

bool Application::initConfiguration() {
  if (!StreamApplication::initConfiguration()) return false;

  try {
    _config.init(this);
  } catch (ValueException &e) {
    SCDETECT_LOG_ERROR("Failed to initialize configuration: %s", e.what());
    return false;
  }

  return true;
}

bool Application::init() {
  if (!StreamApplication::init()) return false;

  _outputOrigins = addOutputObjectLog("origin", primaryMessagingGroup());
  _outputAmplitudes =
      addOutputObjectLog("amplitude", _config.amplitudeMessagingGroup);

  if (_config.playbackConfig.enabled) {
    SCDETECT_LOG_INFO("Playback mode enabled");
  }

  // load event related data
  if (!loadEvents(_config.urlEventDb, query())) {
    SCDETECT_LOG_ERROR("Failed to load events");
    return false;
  }

  // TODO(damb): Check if std::unique_ptr wouldn't be sufficient, here.
  WaveformHandlerIfacePtr waveformHandler{
      util::make_smart<WaveformHandler>(recordStreamURL())};
  if (!_config.templatesNoCache) {
    // cache template waveforms on filesystem
    _config.pathFilesystemCache =
        boost::filesystem::path(_config.pathFilesystemCache).string();
    if (!Util::pathExists(_config.pathFilesystemCache) &&
        !Util::createPath(_config.pathFilesystemCache)) {
      SCDETECT_LOG_ERROR("Failed to create path (waveform cache): %s",
                         _config.pathFilesystemCache.c_str());
      return false;
    }

    waveformHandler = util::make_smart<FileSystemCache>(
        waveformHandler, _config.pathFilesystemCache,
        settings::kCacheRawWaveforms);
  }
  // cache demeaned template waveform snippets in order to speed up the
  // initialization procedure
  waveformHandler =
      util::make_smart<InMemoryCache>(waveformHandler, /*raw=*/false);

  // load template related data
  // TODO(damb):
  //
  // - Allow parsing template configuration from profiles
  if (_config.pathTemplateJson.empty()) {
    SCDETECT_LOG_ERROR("Missing template configuration file.");
    return false;
  }

  TemplateConfigs templateConfigs;
  SCDETECT_LOG_INFO("Loading template configuration from %s",
                    _config.pathTemplateJson.c_str());
  try {
    std::ifstream ifs{_config.pathTemplateJson};
    if (!initDetectors(ifs, waveformHandler.get(), templateConfigs)) {
      return false;
    }
  } catch (std::ifstream::failure &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        _config.pathTemplateJson.c_str(), e.what());
    return false;
  }

  // load bindings
  _bindings.setDefault(_config.sensorLocationBindings);
  if (configModule()) {
    SCDETECT_LOG_DEBUG("Loading binding configuration");
    _bindings.load(&configuration(), configModule(), name());
  }

  bool magnitudesForcedDisabled{_config.magnitudesForceMode &&
                                !*_config.magnitudesForceMode};
  // TODO (damb):
  // - disable magnitude processor setup in case of either magntiude
  // calculation is disabled or magnitude calculation is disabled for all
  // detectors or rather sensor location bindings
  //
  // optionally configure magnitude processors
  if (!magnitudesForcedDisabled) {
    if (_config.pathTemplateFamilyJson.empty()) {
      SCDETECT_LOG_ERROR("Missing template family configuration file.");
      return false;
    }

    SCDETECT_LOG_INFO("Loading template family configuration from %s",
                      _config.pathTemplateJson.c_str());
    try {
      std::ifstream ifs{_config.pathTemplateFamilyJson};
      // TODO(damb):
      // - which waveform handler to be used
      if (!initTemplateFamilies(ifs, waveformHandler.get(), templateConfigs)) {
        return false;
      }
    } catch (std::ifstream::failure &e) {
      SCDETECT_LOG_ERROR(
          "Failed to parse JSON template configuration file (%s): %s",
          _config.pathTemplateJson.c_str(), e.what());
      return false;
    }

    initAmplitudeProcessorFactory();
    initMagnitudeProcessorFactories();
  }

  // free memory after initialization
  EventStore::Instance().reset();

  return true;
}

bool Application::run() {
  SCDETECT_LOG_DEBUG("Application initialized");

  if (_config.templatesPrepare) {
    SCDETECT_LOG_DEBUG(
        "Requested application exit after template initialization");
    return true;
  }

  if (!_detectors.empty()) {
    if (commandline().hasOption("ep")) {
      _ep = util::make_smart<DataModel::EventParameters>();
    }

    SCDETECT_LOG_DEBUG(
        "Subscribing to streams required for detection processing");
    auto cmp = [](const util::HorizontalComponents &lhs,
                  const util::HorizontalComponents &rhs) {
      return util::getWaveformStreamId(lhs) < util::getWaveformStreamId(rhs);
    };
    std::set<util::HorizontalComponents, decltype(cmp)>
        uniqueHorizontalComponents{cmp};
    Core::Time amplitudeStreamsSubscriptionTime{
        _config.playbackConfig.startTimeStr.empty()
            ? Core::Time::GMT()
            : _config.playbackConfig.startTime};
    for (const auto &detectorPair : _detectors) {
      util::WaveformStreamID waveformStreamId{detectorPair.first};

      recordStream()->addStream(
          waveformStreamId.netCode(), waveformStreamId.staCode(),
          waveformStreamId.locCode(), waveformStreamId.chaCode());

      if (detectorPair.second->publishConfig().createAmplitudes ||
          detectorPair.second->publishConfig().createMagnitudes) {
        try {
          uniqueHorizontalComponents.emplace(util::HorizontalComponents{
              Client::Inventory::Instance(), waveformStreamId.netCode(),
              waveformStreamId.staCode(), waveformStreamId.locCode(),
              waveformStreamId.chaCode(), amplitudeStreamsSubscriptionTime});
        } catch (Exception &e) {
          SCDETECT_LOG_WARNING(
              "%s.%s.%s.%s: %s. Skipping amplitude calculation.",
              waveformStreamId.netCode().c_str(),
              waveformStreamId.staCode().c_str(),
              waveformStreamId.locCode().c_str(),
              waveformStreamId.chaCode().c_str(), e.what());
          continue;
        }
      }
    }

    if (!uniqueHorizontalComponents.empty()) {
      SCDETECT_LOG_DEBUG(
          "Subscribing to streams required for amplitude calculation");
      for (const auto &horizontalComponents : uniqueHorizontalComponents) {
        for (auto c : horizontalComponents) {
          recordStream()->addStream(horizontalComponents.netCode(),
                                    horizontalComponents.staCode(),
                                    horizontalComponents.locCode(), c->code());
        }
      }
    }

    if (!_config.playbackConfig.startTimeStr.empty()) {
      recordStream()->setStartTime(_config.playbackConfig.startTime);
      _config.playbackConfig.enabled = true;
    }
    if (!_config.playbackConfig.endTimeStr.empty()) {
      recordStream()->setEndTime(_config.playbackConfig.endTime);
      _config.playbackConfig.enabled = true;
    }
  }

  return StreamApplication::run();
}

void Application::done() {
  if (!_config.templatesPrepare) {
    std::unordered_set<std::string> detectorIds;
    // terminate detectors
    for (const auto &detectorPair : _detectors) {
      auto &detector{detectorPair.second};
      const auto detectorId{detector->id()};
      if (detectorIds.find(detectorId) == detectorIds.end()) {
        detectorIds.emplace(detector->id());
        detector->terminate();
      }
    }

    // flush pending detections
    for (auto &detectionPair : _detections) {
      publishAndRemoveDetection(detectionPair.second);
    }

    if (_ep) {
      IO::XMLArchive ar;
      ar.create(_config.pathEp.empty() ? "-" : _config.pathEp.c_str());
      ar.setFormattedOutput(true);
      ar << _ep;
      ar.close();
      SCDETECT_LOG_DEBUG("Found %lu origins.", _ep->originCount());
      _ep.reset();
    }
  }

  EventStore::Instance().reset();
  RecordResamplerStore::Instance().reset();

  StreamApplication::done();
}

void Application::handleRecord(Record *rec) {
  if (!rec->data()) return;

  if (!_waveformBuffer.feed(rec)) return;

  auto detectorRange{_detectors.equal_range(std::string{rec->streamID()})};
  for (auto it = detectorRange.first; it != detectorRange.second; ++it) {
    auto &detector{it->second};
    if (detector->enabled()) {
      if (!detector->feed(rec)) {
        SCDETECT_LOG_WARNING(
            "%s: Failed to feed record into detector (id=%s). Resetting.",
            it->first.c_str(), detector->id().c_str());
        detector->reset();
        continue;
      }

      if (detector->finished()) {
        SCDETECT_LOG_WARNING(
            "%s: Detector (id=%s) finished (status=%d, "
            "statusValue=%f). Resetting.",
            it->first.c_str(), detector->id().c_str(),
            util::asInteger(detector->status()), detector->statusValue());
        detector->reset();
        continue;
      }
    } else {
      SCDETECT_LOG_DEBUG(
          "%s: Skip feeding record into detector (id=%s). Reason: Disabled.",
          it->first.c_str(), detector->id().c_str());
    }
  }

  _amplitudeProcessorRegistrationBlocked = true;

  auto amplitudeProcessorRange{
      _amplitudeProcessors.equal_range(rec->streamID())};
  for (auto it = amplitudeProcessorRange.first;
       it != amplitudeProcessorRange.second; ++it) {
    const auto &proc{it->second};
    // the amplitude processor must not be already on the removal list
    if (std::find_if(
            std::begin(_amplitudeProcessorRemovalQueue),
            std::end(_amplitudeProcessorRemovalQueue),
            [&proc](const decltype(_amplitudeProcessorRemovalQueue)::value_type
                        &item) { return item.amplitudeProcessor == proc; }) !=
        _amplitudeProcessorRemovalQueue.end()) {
      continue;
    }

    // schedule the amplitude processor for deletion when finished
    if (it->second->finished()) {
      removeAmplitudeProcessor(it->second);
    } else {
      it->second->feed(rec);
      if (it->second->finished()) {
        removeAmplitudeProcessor(it->second);
      }
    }
  }

  _amplitudeProcessorRegistrationBlocked = false;

  // remove outdated amplitude processors
  while (!_amplitudeProcessorRemovalQueue.empty()) {
    const auto amplitudeProcessor{
        _amplitudeProcessorRemovalQueue.front().amplitudeProcessor};
    _amplitudeProcessorRemovalQueue.pop_front();
    removeAmplitudeProcessor(amplitudeProcessor);
  }

  // register pending amplitude processors
  while (!_amplitudeProcessorQueue.empty()) {
    const auto &amplitudeProcessorItem{_amplitudeProcessorQueue.front()};
    _amplitudeProcessorQueue.pop_front();
    registerAmplitudeProcessor(amplitudeProcessorItem.waveformStreamIds,
                               amplitudeProcessorItem.amplitudeProcessor);
  }

  _detectionRegistrationBlocked = true;

  auto detectionRange{_detections.equal_range(rec->streamID())};
  for (auto it = detectionRange.first; it != detectionRange.second; ++it) {
    auto &detection{it->second};
    // the detection must not be already in the removal list
    if (std::find(std::begin(_detectionRemovalQueue),
                  std::end(_detectionRemovalQueue),
                  detection) != std::end(_detectionRemovalQueue)) {
      continue;
    }

    // schedule the detection for deletion when finished
    if (detection->ready()) {
      publishAndRemoveDetection(detection);
    }
  }

  _detectionRegistrationBlocked = false;

  // remove outdated detections
  while (!_detectionRemovalQueue.empty()) {
    auto detection{_detectionRemovalQueue.front()};
    _detectionRemovalQueue.pop_front();
    publishAndRemoveDetection(detection);
  }

  // register pending detections
  while (!_detectionQueue.empty()) {
    const auto detection{_detectionQueue.front()};
    _detectionQueue.pop_front();
    registerDetection(detection);
  }
}

bool Application::isEventDatabaseEnabled() const {
  return _config.urlEventDb.empty();
}

void Application::processDetection(
    const detector::DetectorWaveformProcessor *processor, const Record *record,
    const detector::DetectorWaveformProcessor::DetectionCPtr &detection) {
  SCDETECT_LOG_DEBUG_PROCESSOR(
      processor,
      "Start processing detection (time=%s, associated_results=%d) ...",
      detection->time.iso().c_str(), detection->templateResults.size());

  Core::Time now{Core::Time::GMT()};

  DataModel::CreationInfo ci;
  ci.setAgencyID(agencyID());
  ci.setAuthor(author());
  ci.setCreationTime(now);

  DataModel::OriginPtr origin{DataModel::Origin::Create()};
  if (!origin) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        processor, "Internal error: duplicate origin identifier");
    return;
  }

  {
    auto comment{util::make_smart<DataModel::Comment>()};
    comment->setId(settings::kDetectorIdCommentId);
    comment->setText(processor->id());
    origin->add(comment.get());
  }
  {
    auto comment{util::make_smart<DataModel::Comment>()};
    comment->setId("scdetectResultCCC");
    comment->setText(std::to_string(detection->fit));
    origin->add(comment.get());
  }

  origin->setCreationInfo(ci);
  origin->setLatitude(DataModel::RealQuantity(detection->latitude));
  origin->setLongitude(DataModel::RealQuantity(detection->longitude));
  origin->setDepth(DataModel::RealQuantity(detection->depth));
  origin->setTime(DataModel::TimeQuantity(detection->time));
  origin->setMethodID(detection->publishConfig.originMethodId);
  origin->setEpicenterFixed(true);
  origin->setEvaluationMode(DataModel::EvaluationMode(DataModel::AUTOMATIC));

  std::vector<double> azimuths;
  std::vector<double> distances;
  for (const auto &resultPair : detection->templateResults) {
    double az, baz, dist;
    const auto &sensorLocation{resultPair.second.sensorLocation};
    Math::Geo::delazi(detection->latitude, detection->longitude,
                      sensorLocation.latitude, sensorLocation.longitude, &dist,
                      &az, &baz);

    distances.push_back(dist);
    azimuths.push_back(az);
  }

  std::sort(azimuths.begin(), azimuths.end());
  std::sort(distances.begin(), distances.end());

  DataModel::OriginQuality originQuality;
  if (azimuths.size() > 2) {
    double azGap{};
    for (size_t i = 0; i < azimuths.size() - 1; ++i)
      azGap = (azimuths[i + 1] - azimuths[i]) > azGap
                  ? (azimuths[i + 1] - azimuths[i])
                  : azGap;

    originQuality.setAzimuthalGap(azGap);
  }

  if (!distances.empty()) {
    originQuality.setMinimumDistance(distances.front());
    originQuality.setMaximumDistance(distances.back());
    originQuality.setMedianDistance(distances[distances.size() / 2]);
  }

  originQuality.setStandardError(1.0 - detection->fit);
  originQuality.setAssociatedStationCount(detection->numStationsAssociated);
  originQuality.setUsedStationCount(detection->numStationsUsed);
  originQuality.setAssociatedPhaseCount(detection->numChannelsAssociated);
  originQuality.setUsedPhaseCount(detection->numChannelsUsed);

  origin->setQuality(originQuality);

  DetectionItem detectionItem{origin};
  detectionItem.detectorId = processor->id();
  detectionItem.detection = detection;

  detectionItem.config = DetectionItem::ProcessorConfig{
      processor->gapInterpolation(), processor->gapThreshold(),
      processor->gapTolerance()};

  const auto createPick = [](const detector::Arrival &arrival,
                             bool asTemplateArrivalPick) {
    DataModel::PickPtr ret{DataModel::Pick::Create()};
    if (!ret) {
      throw DuplicatePublicObjectId{"duplicate pick identifier"};
    }

    ret->setTime(DataModel::TimeQuantity{arrival.pick.time, boost::none,
                                         arrival.pick.lowerUncertainty,
                                         arrival.pick.upperUncertainty});
    util::WaveformStreamID waveformStreamId{arrival.pick.waveformStreamId};
    if (asTemplateArrivalPick) {
      ret->setWaveformID(DataModel::WaveformStreamID{
          waveformStreamId.netCode(), waveformStreamId.staCode(),
          waveformStreamId.locCode(), waveformStreamId.chaCode(), ""});
    } else {
      ret->setWaveformID(DataModel::WaveformStreamID{
          waveformStreamId.netCode(), waveformStreamId.staCode(),
          waveformStreamId.locCode(),
          util::getBandAndSourceCode(waveformStreamId), ""});
    }
    ret->setEvaluationMode(DataModel::EvaluationMode(DataModel::AUTOMATIC));

    if (arrival.pick.phaseHint) {
      ret->setPhaseHint(DataModel::Phase{*arrival.pick.phaseHint});
    }
    return ret;
  };

  const auto createArrival = [&ci](const detector::Arrival &arrival,
                                   const DataModel::PickCPtr &pick) {
    auto ret{util::make_smart<DataModel::Arrival>()};
    if (!ret) {
      throw DuplicatePublicObjectId{"duplicate arrival identifier"};
    }
    ret->setCreationInfo(ci);
    ret->setPickID(pick->publicID());
    ret->setPhase(arrival.phase);
    if (arrival.weight) {
      ret->setWeight(arrival.weight);
    }
    return ret;
  };

  auto createPicks{detection->publishConfig.createArrivals ||
                   detection->publishConfig.createAmplitudes ||
                   detection->publishConfig.createMagnitudes};
  if (createPicks) {
    using PhaseCode = std::string;
    using ProcessedPhaseCodes = std::unordered_set<PhaseCode>;
    using SensorLocationStreamId = std::string;
    using SensorLocationStreamIdProcessedPhaseCodesMap =
        std::unordered_map<SensorLocationStreamId, ProcessedPhaseCodes>;
    SensorLocationStreamIdProcessedPhaseCodesMap processedPhaseCodes;

    for (const auto &resultPair : detection->templateResults) {
      const auto &res{resultPair.second};

      auto sensorLocationStreamId{util::getSensorLocationStreamId(
          util::WaveformStreamID{res.arrival.pick.waveformStreamId})};

      try {
        const auto pick{createPick(res.arrival, false)};
        if (!pick->add(createTemplateWaveformTimeInfoComment(res).release())) {
          SCDETECT_LOG_WARNING_PROCESSOR(processor,
                                         "Internal error: failed to add "
                                         "template waveform time info comment");
        }

        auto &processed{processedPhaseCodes[sensorLocationStreamId]};
        auto phaseAlreadyProcessed{
            std::find(std::begin(processed), std::end(processed),
                      res.arrival.phase) != std::end(processed)};
        // XXX(damb): assign a phase only once per sensor location
        if (detection->publishConfig.createArrivals && !phaseAlreadyProcessed) {
          const auto arrival{createArrival(res.arrival, pick)};
          detectionItem.arrivalPicks.push_back({arrival, pick});
          processed.emplace(res.arrival.phase);
        }

        if (detection->publishConfig.createAmplitudes ||
            detection->publishConfig.createMagnitudes) {
          detectionItem.amplitudePickMap.emplace(
              resultPair.first,
              DetectionItem::Pick{res.arrival.pick.waveformStreamId, pick});
        }
      } catch (DuplicatePublicObjectId &e) {
        SCDETECT_LOG_WARNING_PROCESSOR(processor, "Internal error: %s",
                                       e.what());
        continue;
      }
    }
  }

  // create theoretical template arrivals
  if (detection->publishConfig.createTemplateArrivals) {
    for (const auto &a : detection->publishConfig.theoreticalTemplateArrivals) {
      try {
        const auto pick{createPick(a, true)};
        const auto arrival{createArrival(a, pick)};
        detectionItem.arrivalPicks.push_back({arrival, pick});
      } catch (DuplicatePublicObjectId &e) {
        SCDETECT_LOG_WARNING_PROCESSOR(processor, "Internal error: %s",
                                       e.what());
        continue;
      }
    }
  }

  auto magnitudeForcedEnabled{_config.magnitudesForceMode &&
                              *_config.magnitudesForceMode};
  auto amplitudeForcedEnabled{
      (_config.amplitudesForceMode && *_config.amplitudesForceMode) ||
      magnitudeForcedEnabled};
  auto amplitudeForcedDisabled{
      (_config.amplitudesForceMode && !*_config.amplitudesForceMode) &&
      !magnitudeForcedEnabled};

  if (amplitudeForcedEnabled ||
      (!amplitudeForcedDisabled && detection->publishConfig.createAmplitudes)) {
    // XXX(damb): as soon as either amplitudes or magnitudes need to be
    // computed, the detection is issued as a wholesale due to simplicity. (Note
    // that the amplitudes could be issued independently from the origin while
    // magnitudes need to be associated to the origin.)
    auto detectionItemPtr{
        std::make_shared<DetectionItem>(std::move(detectionItem))};
    registerDetection(detectionItemPtr);

    initAmplitudeProcessors(detectionItemPtr, *processor);
  } else {
    publishDetection(detectionItem);
  }
}

void Application::publishDetection(const DetectionItem &detectionItem) {
  logObject(_outputOrigins, Core::Time::GMT());

  if (connection() && !_config.noPublish) {
    SCDETECT_LOG_DEBUG_TAGGED(detectionItem.detectorId,
                              "Sending event parameters (detection) ...");

    auto notifierMsg{util::make_smart<DataModel::NotifierMessage>()};

    // origin
    auto notifier{util::make_smart<DataModel::Notifier>(
        "EventParameters", DataModel::OP_ADD, detectionItem.origin.get())};
    notifierMsg->attach(notifier.get());

    for (auto &arrivalPick : detectionItem.arrivalPicks) {
      // pick
      {
        auto notifier{util::make_smart<DataModel::Notifier>(
            "EventParameters", DataModel::OP_ADD, arrivalPick.pick.get())};

        notifierMsg->attach(notifier.get());
      }
      // arrival
      {
        auto notifier{util::make_smart<DataModel::Notifier>(
            detectionItem.origin->publicID(), DataModel::OP_ADD,
            arrivalPick.arrival.get())};

        notifierMsg->attach(notifier.get());
      }
    }

    // station magnitudes
    for (auto &mag : detectionItem.magnitudes) {
      auto notifier{util::make_smart<DataModel::Notifier>(
          detectionItem.origin->publicID(), DataModel::OP_ADD, mag.get())};

      notifierMsg->attach(notifier.get());
    }

    if (!connection()->send(notifierMsg.get())) {
      SCDETECT_LOG_ERROR_TAGGED(
          detectionItem.detectorId,
          "Sending of event parameters (detection) failed.");
    }
  }

  if (_ep) {
    _ep->add(detectionItem.origin.get());

    for (auto &arrivalPick : detectionItem.arrivalPicks) {
      detectionItem.origin->add(arrivalPick.arrival.get());

      _ep->add(arrivalPick.pick.get());
    }

    for (auto &mag : detectionItem.magnitudes) {
      detectionItem.origin->add(mag.get());
    }
  }

  // amplitudes
  for (auto &amp : detectionItem.amplitudes) {
    logObject(_outputAmplitudes, Core::Time::GMT());
    if (connection() && !_config.noPublish) {
      SCDETECT_LOG_DEBUG_TAGGED(detectionItem.detectorId,
                                "Sending event parameters (amplitude) ...");

      auto notifierMsg{util::make_smart<DataModel::NotifierMessage>()};
      auto notifier{util::make_smart<DataModel::Notifier>(
          "EventParameters", DataModel::OP_ADD, amp.get())};
      notifierMsg->attach(notifier.get());

      if (!connection()->send(_config.amplitudeMessagingGroup,
                              notifierMsg.get())) {
        SCDETECT_LOG_ERROR_TAGGED(
            detectionItem.detectorId,
            "Sending of event parameters (amplitude) failed.");
      }
    }

    if (_ep) {
      _ep->add(amp.get());
    }
  }
}

DataModel::AmplitudePtr Application::createAmplitude(
    const AmplitudeProcessor *processor, const Record *record,
    const AmplitudeProcessor::AmplitudeCPtr &amplitude,
    const boost::optional<std::string> &methodId,
    const boost::optional<std::string> &amplitudeType) {
  Core::TimeWindow tw{
      amplitude->time.reference - Core::TimeSpan{amplitude->time.begin},
      amplitude->time.reference + Core::TimeSpan{amplitude->time.end}};
  SCDETECT_LOG_DEBUG_TAGGED(
      processor->id(),
      "Creating amplitude (value=%f, starttime=%s, endtime=%s) ...",
      amplitude->value.value, tw.startTime().iso().c_str(),
      tw.endTime().iso().c_str());

  Core::Time now{Core::Time::GMT()};

  DataModel::AmplitudePtr amp{DataModel::Amplitude::Create()};
  if (!amp) {
    throw DuplicatePublicObjectId{"duplicate amplitude identifier"};
  }

  DataModel::CreationInfo ci;
  ci.setAgencyID(agencyID());
  ci.setAuthor(author());
  ci.setCreationTime(now);

  amp->setCreationInfo(ci);
  amp->setType(processor->type());
  amp->setUnit(processor->unit());

  if (amplitude->waveformStreamIds) {
    if (amplitude->waveformStreamIds.value().size() == 1) {
      const util::WaveformStreamID waveformStreamId{
          amplitude->waveformStreamIds.value()[0]};
      amp->setWaveformID(DataModel::WaveformStreamID{
          waveformStreamId.netCode(), waveformStreamId.staCode(),
          waveformStreamId.locCode(), waveformStreamId.chaCode(), ""});
    }
  }

  amp->setAmplitude(DataModel::RealQuantity{
      amplitude->value.value, boost::none, amplitude->value.lowerUncertainty,
      amplitude->value.upperUncertainty, boost::none});
  amp->setTimeWindow(DataModel::TimeWindow{
      amplitude->time.reference, amplitude->time.begin, amplitude->time.end});
  amp->setMethodID(methodId.value_or(""));

  amp->setSnr(amplitude->snr);
  if (amplitude->dominantPeriod) {
    amp->setPeriod(DataModel::RealQuantity{*amplitude->dominantPeriod});
  }
  processor->finalize(amp.get());

  if (amplitudeType) {
    amp->setType(*amplitudeType);
  }

  return amp;
}

bool Application::initAmplitudeProcessorFactory() {
  AmplitudeProcessor::Factory::registerFactory(
      "MLx", AmplitudeProcessor::Factory::createMLx);
  AmplitudeProcessor::Factory::registerFactory(
      "MRelative", AmplitudeProcessor::Factory::createMRelative);

  return true;
}

bool Application::initMagnitudeProcessorFactories() {
  MagnitudeProcessor::Factory::registerFactory("MLx", []() {
    return util::make_unique<magnitude::MLxFixedSlopeRegressionMagnitude>();
  });

  return true;
}

bool Application::loadEvents(const std::string &eventDb,
                             DataModel::DatabaseQueryPtr db) {
  bool loaded{false};
  if (!eventDb.empty()) {
    SCDETECT_LOG_INFO("Loading events from %s", eventDb.c_str());

    auto loadFromFile = [this, &loaded](const std::string &path) {
      try {
        EventStore::Instance().load(path);
        loaded = true;
      } catch (std::exception &e) {
        auto msg{Core::stringify("Failed to load events: %s", e.what())};
        if (isDatabaseEnabled()) {
          SCDETECT_LOG_WARNING("%s", msg.c_str());
        } else {
          SCDETECT_LOG_ERROR("%s", msg.c_str());
        }
      }
    };

    if (eventDb.find("://") == std::string::npos) {
      loadFromFile(eventDb);
    } else if (eventDb.find("file://") == 0) {
      loadFromFile(eventDb.substr(7));
    } else {
      SCDETECT_LOG_INFO("Trying to connect to %s", eventDb.c_str());
      IO::DatabaseInterfacePtr db{IO::DatabaseInterface::Open(eventDb.c_str())};
      if (db) {
        SCDETECT_LOG_INFO("Connected successfully");
        auto query{util::make_smart<DataModel::DatabaseQuery>(db.get())};
        EventStore::Instance().load(query.get());
        loaded = true;
      } else {
        SCDETECT_LOG_WARNING("Database connection to %s failed",
                             eventDb.c_str());
      }
    }
  }

  if (!loaded && isDatabaseEnabled()) {
    SCDETECT_LOG_INFO("Loading events from %s", databaseURI().c_str());
    try {
      EventStore::Instance().load(query());
      loaded = true;
    } catch (std::exception &e) {
      SCDETECT_LOG_ERROR("Failed to load events: %s", e.what());
    }
  }

  if (loaded) {
    SCDETECT_LOG_INFO("Finished loading events");
  }

  return loaded;
}

bool Application::initDetectors(std::ifstream &ifs,
                                WaveformHandlerIface *waveformHandler,
                                TemplateConfigs &templateConfigs) {
  // initialize detectors
  struct TemplateProcessorIds {
    bool complete{false};
    std::unordered_set<std::string> ids;
  };
  std::unordered_map<std::string, TemplateProcessorIds> processorIds;
  auto isUniqueProcessorId = [&processorIds](const std::string &detectorId,
                                             const std::string &templateId) {
    auto it{processorIds.find(detectorId)};
    bool detectorExists{it != processorIds.end()};
    if (detectorExists) {
      if (it->second.complete) {
        SCDETECT_LOG_WARNING(
            "Processor id is be used by multiple "
            "processors: detectorId=%s",
            detectorId.c_str());
        return false;
      } else {
        bool idExists{it->second.ids.find(templateId) != it->second.ids.end()};
        if (idExists) {
          SCDETECT_LOG_WARNING(
              "Processor id is be used by multiple processors: "
              "detectorId=%s, templateId=%s",
              detectorId.c_str(), templateId.c_str());
        }
        it->second.ids.emplace(templateId);
        return !idExists;
      }
    } else {
      TemplateProcessorIds templateIds;
      templateIds.ids.emplace(templateId);
      processorIds.emplace(detectorId, templateIds);
      return !detectorExists;
    }
  };

  try {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ifs, pt);

    for (const auto &templateSettingPt : pt) {
      try {
        config::TemplateConfig tc{templateSettingPt.second,
                                  _config.detectorConfig, _config.streamConfig,
                                  _config.publishConfig};

        SCDETECT_LOG_DEBUG("Creating detector processor (id=%s) ... ",
                           tc.detectorId().c_str());

        auto detectorBuilder{
            std::move(detector::DetectorWaveformProcessor::Create(tc.originId())
                          .setId(tc.detectorId())
                          .setConfig(tc.publishConfig(), tc.detectorConfig(),
                                     _config.playbackConfig.enabled)
                          .setEventParameters())};

        std::vector<std::string> streamIds;
        for (const auto &streamConfigPair : tc) {
          isUniqueProcessorId(tc.detectorId(),
                              streamConfigPair.second.templateId);
          try {
            detectorBuilder.setStream(streamConfigPair.first,
                                      streamConfigPair.second, waveformHandler);
          } catch (builder::NoSensorLocation &e) {
            if (_config.skipTemplateIfNoSensorLocationData) {
              SCDETECT_LOG_WARNING(
                  "%s. Skipping template waveform processor initialization.",
                  e.what());
              continue;
            }
            throw;
          } catch (builder::NoStream &e) {
            if (_config.skipTemplateIfNoStreamData) {
              SCDETECT_LOG_WARNING(
                  "%s. Skipping template waveform processor initialization.",
                  e.what());
              continue;
            }
            throw;
          } catch (builder::NoPick &e) {
            if (_config.skipTemplateIfNoPick) {
              SCDETECT_LOG_WARNING(
                  "%s. Skipping template waveform processor initialization.",
                  e.what());
              continue;
            }
            throw;
          } catch (builder::NoWaveformData &e) {
            if (_config.skipTemplateIfNoWaveformData) {
              SCDETECT_LOG_WARNING(
                  "%s. Skipping template waveform processor initialization.",
                  e.what());
              continue;
            }
            throw;
          }
          streamIds.push_back(streamConfigPair.first);
        }
        processorIds.at(tc.detectorId()).complete = true;

        std::shared_ptr<detector::DetectorWaveformProcessor> detectorPtr{
            detectorBuilder.build()};
        detectorPtr->setResultCallback(
            [this](
                const detector::DetectorWaveformProcessor *processor,
                const Record *record,
                detector::DetectorWaveformProcessor::DetectionCPtr detection) {
              processDetection(processor, record, detection);
            });

        for (const auto &streamId : streamIds)
          _detectors.emplace(streamId, detectorPtr);

        templateConfigs.push_back(tc);

      } catch (Exception &e) {
        SCDETECT_LOG_WARNING("Failed to create detector: %s. Skipping.",
                             e.what());
        continue;
      }
    }
  } catch (boost::property_tree::json_parser::json_parser_error &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        _config.pathTemplateJson.c_str(), e.what());
    return false;
  } catch (std::ifstream::failure &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        _config.pathTemplateJson.c_str(), e.what());
    return false;
  } catch (std::exception &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        _config.pathTemplateJson.c_str(), e.what());
    return false;
  } catch (...) {
    SCDETECT_LOG_ERROR("Failed to parse JSON template configuration file (%s)",
                       _config.pathTemplateJson.c_str());
    return false;
  }
  return true;
}

bool Application::initAmplitudeProcessors(
    std::shared_ptr<DetectionItem> &detectionItem,
    const detector::DetectorWaveformProcessor &detectorProcessor) {
  using PickMap = std::pair<DetectionItem::ProcessorId, DetectionItem::Pick>;
  using SensorLocationPickMap = std::multimap<std::string, PickMap>;
  SensorLocationPickMap sensorLocationPickMap;
  // gather unique sensor locations which have amplitude calculation enabled
  // (requires bindings to be configured)
  for (const auto &pickMapPair : detectionItem->amplitudePickMap) {
    const auto &processorId{pickMapPair.first};
    const auto &pick{pickMapPair.second};
    const util::WaveformStreamID waveformStreamId{
        pick.authorativeWaveformStreamId};
    try {
      auto hasAmplitudeCalculationEnabled{
          _bindings
              .at(waveformStreamId.netCode(), waveformStreamId.staCode(),
                  waveformStreamId.locCode(), waveformStreamId.chaCode())
              .amplitudeProcessingConfig.enabled};
      if (hasAmplitudeCalculationEnabled) {
        sensorLocationPickMap.emplace(
            util::getSensorLocationStreamId(waveformStreamId, true),
            std::make_pair(processorId, pick));
      }
    } catch (std::out_of_range &) {
      continue;
    }
  }

  for (const auto &sensorLocationPickMapPair : sensorLocationPickMap) {
    const auto &sensorLocationStreamId{sensorLocationPickMapPair.first};
    std::vector<std::string> sensorLocationStreamIdTokens;
    util::tokenizeWaveformStreamId(sensorLocationStreamId,
                                   sensorLocationStreamIdTokens);

    auto &sensorLocationBindings{_bindings.at(
        sensorLocationStreamIdTokens[0], sensorLocationStreamIdTokens[1],
        sensorLocationStreamIdTokens[2], sensorLocationStreamIdTokens[3])};
    auto &amplitudeTypes{
        sensorLocationBindings.amplitudeProcessingConfig.amplitudeTypes};

    for (const auto &amplitudeType : amplitudeTypes) {
      std::vector<std::string> waveformStreamIds;

      amplitude::factory::Detection detection;
      detection.origin = detectionItem->origin;
      detection.sensorLocationStreamId = sensorLocationStreamId;

      auto range{sensorLocationPickMap.equal_range(sensorLocationStreamId)};
      for (auto i{range.first}; i != range.second; ++i) {
        const auto &pickPair{i->second};
        waveformStreamIds.push_back(
            pickPair.second.authorativeWaveformStreamId);

        detection.pickMap.emplace(
            pickPair.first, amplitude::factory::Detection::Pick{
                                pickPair.second.authorativeWaveformStreamId,
                                pickPair.second.pick});
      }

      auto amplitudeProcessor{AmplitudeProcessor::Factory::create(
          amplitudeType, _bindings, detection, detectorProcessor)};

      // configure callback
      bool magnitudesForcedEnabled{_config.magnitudesForceMode &&
                                   *_config.magnitudesForceMode};
      bool magnitudesForcedDisabled{_config.magnitudesForceMode &&
                                    !*_config.magnitudesForceMode};

      const auto &magnitudeProcessingConfig{
          sensorLocationBindings.magnitudeProcessingConfig};
      const auto magnitudeType{amplitudeType};
      bool magnitudeCalculationEnabled{
          magnitudesForcedEnabled ||
          (!magnitudesForcedDisabled &&
           detectorProcessor.publishConfig().createMagnitudes &&
           magnitudeProcessingConfig.enabled &&
           std::find(std::begin(magnitudeProcessingConfig.magnitudeTypes),
                     std::end(magnitudeProcessingConfig.magnitudeTypes),
                     magnitudeType) !=
               std::end(magnitudeProcessingConfig.magnitudeTypes))};
      auto magnitudeProcessorId{detectorProcessor.id() +
                                settings::kProcessorIdSep + util::createUUID()};

      ++detectionItem->numberOfRequiredAmplitudes;

      amplitudeProcessor->setResultCallback(
          [this, detectionItem, magnitudeType, magnitudeCalculationEnabled,
           magnitudeProcessorId](const AmplitudeProcessor *processor,
                                 const Record *record,
                                 AmplitudeProcessor::AmplitudeCPtr result) {
            assert(processor);
            DataModel::AmplitudePtr amplitude;
            // create amplitude
            try {
              amplitude = createAmplitude(processor, record, result,
                                          boost::none, magnitudeType);

              if (!amplitude) {
                --detectionItem->numberOfRequiredAmplitudes;
                return;
              }
            } catch (Exception &e) {
              --detectionItem->numberOfRequiredAmplitudes;
              SCDETECT_LOG_WARNING_PROCESSOR(
                  processor, "Failed to create amplitude: %s", e.what());
            }

            detectionItem->amplitudes.emplace_back(amplitude);

            if (magnitudeCalculationEnabled) {
              ++detectionItem->numberOfRequiredMagnitudes;
              // create station magnitude
              try {
                auto mag{createMagnitude(amplitude.get(), magnitudeType, "",
                                         magnitudeProcessorId)};
                if (!mag) {
                  --detectionItem->numberOfRequiredMagnitudes;
                  return;
                }

                detectionItem->magnitudes.emplace_back(mag);

                SCDETECT_LOG_DEBUG_TAGGED(
                    magnitudeProcessorId,
                    "Created station magnitude for origin (%s): public_id=%s, "
                    "type=%s",
                    detectionItem->origin->publicID().c_str(),
                    mag->publicID().c_str(), mag->type().c_str());

              } catch (Exception &e) {
                --detectionItem->numberOfRequiredMagnitudes;
                SCDETECT_LOG_WARNING_TAGGED(
                    magnitudeProcessorId,
                    "Failed to create station magnitude: %s", e.what());
              }
            }
          });

      try {
        registerAmplitudeProcessor(waveformStreamIds,
                                   std::move(amplitudeProcessor));
      } catch (Exception &e) {
        SCDETECT_LOG_WARNING("Failed to register amplitude processor: %s",
                             e.what());
        continue;
      }
    }
  }

  return true;
}

void Application::publishAndRemoveDetection(
    std::shared_ptr<DetectionItem> &detection) {
  if (!detection->published) {
    publishDetection(*detection);

    detection->published = true;
  }

  removeDetection(detection);
}

bool Application::initTemplateFamilies(std::ifstream &ifs,
                                       WaveformHandlerIface *waveformHandler,
                                       const TemplateConfigs &templateConfigs) {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ifs, pt);

    for (const auto &templateFamilyConfigPair : pt) {
      config::TemplateFamilyConfig tfc(
          templateFamilyConfigPair.second, templateConfigs,
          _config.templateFamilySensorLocationConfig);
      logging::TaggedMessage msg{
          tfc.id(), "Creating template family (id=" + tfc.id() + ") ..."};
      SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());
      try {
        // build and register template family
        auto templateFamily{TemplateFamily::Create(tfc)
                                .setId()
                                .setLimits()
                                .setStationMagnitudes()
                                .setAmplitudes(waveformHandler, _bindings)
                                .build()};
        if (!templateFamily->empty()) {
          MagnitudeProcessor::Factory::registerTemplateFamily(
              std::move(templateFamily));
          msg.setText("Registered template family");
          SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());
        } else {
          msg.setText(
              "Missing template family members. Skipping template family "
              "registration.");
          SCDETECT_LOG_WARNING("%s", logging::to_string(msg).c_str());
        }

      } catch (builder::NoSensorLocation &e) {
        if (_config.skipReferenceConfigIfNoSensorLocationData) {
          SCDETECT_LOG_WARNING("%s. Skipping template family initialization.",
                               e.what());
          continue;
        }
        throw;
      } catch (builder::NoStream &e) {
        if (_config.skipReferenceConfigIfNoStreamData) {
          SCDETECT_LOG_WARNING("%s. Skipping template family initialization.",
                               e.what());
          continue;
        }
        throw;
      } catch (builder::NoPick &e) {
        if (_config.skipReferenceConfigIfNoPick) {
          SCDETECT_LOG_WARNING("%s. Skipping template family initialization.",
                               e.what());
          continue;
        }
        throw;
      } catch (builder::NoBindings &e) {
        if (_config.skipReferenceConfigIfNoBindings) {
          SCDETECT_LOG_WARNING("%s. Skipping template family initialization.",
                               e.what());
          continue;
        }
        throw;
      } catch (builder::NoWaveformData &e) {
        if (_config.skipReferenceConfigIfNoWaveformData) {
          SCDETECT_LOG_WARNING("%s. Skipping template family initialization.",
                               e.what());
          continue;
        }
        throw;
      }
    }

  } catch (config::ValidationError &e) {
    SCDETECT_LOG_ERROR(
        "JSON template family configuration file does not validate (%s): %s",
        _config.pathTemplateFamilyJson.c_str(), e.what());
    return false;
  } catch (config::ParserException &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template family configuration file (%s): %s",
        _config.pathTemplateFamilyJson.c_str(), e.what());
    return false;
  } catch (boost::property_tree::json_parser::json_parser_error &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template family configuration file (%s): %s",
        _config.pathTemplateFamilyJson.c_str(), e.what());
    return false;
  }
  return true;
}

DataModel::StationMagnitudePtr Application::createMagnitude(
    const DataModel::Amplitude *amplitude, const std::string &magnitudeType,
    const std::string &methodId, const std::string &processorId) {
  auto proc{MagnitudeProcessor::Factory::create(amplitude, magnitudeType,
                                                processorId)};
  if (!proc) {
    throw BaseException("failed to create magnitude processor");
  }

  auto magnitudeValue{proc->compute(amplitude)};

  DataModel::StationMagnitudePtr mag{DataModel::StationMagnitude::Create()};
  if (!mag) {
    throw DuplicatePublicObjectId{"duplicate station magnitude identifier"};
  }

  mag->setMagnitude(DataModel::RealQuantity{magnitudeValue});
  mag->setAmplitudeID(amplitude->publicID());
  mag->setMethodID(methodId);

  proc->finalize(mag.get());

  return mag;
}

void Application::registerAmplitudeProcessor(
    const std::vector<WaveformStreamId> &waveformStreamIds,
    const std::shared_ptr<AmplitudeProcessor> &processor) {
  if (_amplitudeProcessorRegistrationBlocked) {
    _amplitudeProcessorQueue.emplace_back(
        AmplitudeProcessorQueueItem{waveformStreamIds, processor});
    return;
  }

  for (const auto &waveformStreamId : waveformStreamIds) {
    _amplitudeProcessors.emplace(waveformStreamId, processor);
    SCDETECT_LOG_DEBUG("[%s] Added amplitude processor with id: %s",
                       waveformStreamId.c_str(), processor->id().c_str());
    SCDETECT_LOG_DEBUG("Current amplitude processor count: %lu",
                       _amplitudeProcessors.size());
  }
  _amplitudeProcessorIdx.emplace(processor->id(), waveformStreamIds);

  for (const auto &waveformStreamId : waveformStreamIds) {
    if (!processor->finished()) {
      util::WaveformStreamID converted{waveformStreamId};
      auto sequence{
          _waveformBuffer.sequence(Processing::StreamBuffer::WaveformID{
              converted.netCode(), converted.staCode(), converted.locCode(),
              converted.chaCode()})};
      if (!sequence) continue;

      const auto tw{processor->safetyTimeWindow()};
      if (tw.startTime() < sequence->timeWindow().startTime()) {
        // TODO:
        // - fetch historical data

        // actually feed as much data as possible
        for (auto it = sequence->begin(); it != sequence->end(); ++it) {
          if ((*it)->startTime() > tw.endTime()) break;
          processor->feed((*it).get());
        }
      } else {
        // find the position in the record sequence to fill the requested
        // time window
        auto rit{sequence->rbegin()};
        while (rit != sequence->rend()) {
          if ((*rit)->endTime() < tw.startTime()) break;
          ++rit;
        }

        RecordSequence::iterator it;
        if (rit == sequence->rend()) {
          it = sequence->begin();
        } else {
          it = --rit.base();
        }
        while (it != sequence->end() && (*it)->startTime() <= tw.endTime()) {
          processor->feed((*it).get());
          ++it;
        }
      }
    }
  }

  if (processor->finished()) {
    removeAmplitudeProcessor(processor);
  }
}

void Application::removeAmplitudeProcessor(
    const std::shared_ptr<AmplitudeProcessor> &processor) {
  if (_amplitudeProcessorRegistrationBlocked) {
    _amplitudeProcessorRemovalQueue.emplace_back(
        AmplitudeProcessorQueueItem{{}, processor});
    return;
  }

  const auto waveformStreamIds{_amplitudeProcessorIdx.at(processor->id())};
  for (const auto &waveformStreamId : waveformStreamIds) {
    auto range{_amplitudeProcessors.equal_range(waveformStreamId)};
    auto it{range.first};
    while (it != range.second) {
      if (it->second == processor) {
        SCDETECT_LOG_DEBUG("[%s] Removed amplitude processor with id: %s",
                           waveformStreamId.c_str(), processor->id().c_str());
        it = _amplitudeProcessors.erase(it);
        SCDETECT_LOG_DEBUG("Current amplitude processor count: %lu",
                           _amplitudeProcessors.size());
      } else {
        ++it;
      }
    }
  }

  _amplitudeProcessorIdx.erase(processor->id());

  // check pending registration queue
  auto it{std::begin(_amplitudeProcessorQueue)};
  while (it != _amplitudeProcessorQueue.end()) {
    if (it->amplitudeProcessor == processor) {
      it = _amplitudeProcessorQueue.erase(it);
      continue;
    }
    ++it;
  }
}

void Application::registerDetection(
    const std::shared_ptr<DetectionItem> &detection) {
  if (_detectionRegistrationBlocked) {
    _detectionQueue.emplace_back(detection);
    return;
  }

  const auto &waveformStreamIds{
      util::map_keys(detection->detection->templateResults)};

  for (const auto &waveformStreamId : waveformStreamIds) {
    _detections.emplace(waveformStreamId, detection);
    SCDETECT_LOG_DEBUG("[%s] Added detection with id: %s",
                       waveformStreamId.c_str(), detection->id().c_str());
    SCDETECT_LOG_DEBUG("Current detection count: %lu", _detections.size());
  }
}

void Application::removeDetection(
    const std::shared_ptr<DetectionItem> &detection) {
  if (_detectionRegistrationBlocked) {
    _detectionRemovalQueue.emplace_back(detection);
    return;
  }

  const auto waveformStreamIds{
      util::map_keys(detection->detection->templateResults)};
  for (const auto &waveformStreamId : waveformStreamIds) {
    auto range{_detections.equal_range(waveformStreamId)};
    auto it{range.first};
    while (it != range.second) {
      if (it->second == detection) {
        SCDETECT_LOG_DEBUG("[%s] Removed detection with id: %s",
                           waveformStreamId.c_str(), detection->id().c_str());
        it = _detections.erase(it);
        SCDETECT_LOG_DEBUG("Current detection count: %lu", _detections.size());
      } else {
        ++it;
      }
    }
  }

  // check pending registration queue
  auto it{std::begin(_detectionQueue)};
  while (it != _detectionQueue.end()) {
    if (*it == detection) {
      it = _detectionQueue.erase(it);
      continue;
    }
    ++it;
  }
}

std::unique_ptr<DataModel::Comment>
Application::createTemplateWaveformTimeInfoComment(
    const detector::DetectorWaveformProcessor::Detection::TemplateResult
        &templateResult) {
  auto ret{util::make_unique<DataModel::Comment>()};
  ret->setId(settings::kTemplateWaveformTimeInfoPickCommentId);
  ret->setText(templateResult.templateWaveformStartTime.iso() +
               settings::kTemplateWaveformTimeInfoPickCommentIdSep +
               templateResult.templateWaveformEndTime.iso() +
               settings::kTemplateWaveformTimeInfoPickCommentIdSep +
               templateResult.templateWaveformReferenceTime.iso());

  return ret;
}

Application::Config::Config() {
  Environment *env{Environment::Instance()};

  boost::filesystem::path scInstallDir{env->installDir()};
  boost::filesystem::path pathCache{scInstallDir /
                                    settings::kPathFilesystemCache};
  pathFilesystemCache = pathCache.string();
}

void Application::Config::init(const Client::Application *app) {
  try {
    pathTemplateJson = app->configGetPath("templatesJSON");
  } catch (...) {
  }
  try {
    const auto messagingGroup{
        app->configGetString("amplitudes.messagingGroup")};
    if (!messagingGroup.empty()) {
      amplitudeMessagingGroup = messagingGroup;
    }
  } catch (...) {
  }

  try {
    publishConfig.createArrivals = app->configGetBool("publish.createArrivals");
  } catch (...) {
  }
  try {
    publishConfig.createTemplateArrivals =
        app->configGetBool("publish.createTemplateArrivals");
  } catch (...) {
  }
  try {
    publishConfig.originMethodId = app->configGetString("publish.methodId");
  } catch (...) {
  }
  try {
    publishConfig.createAmplitudes =
        app->configGetBool("amplitudes.createAmplitudes");
  } catch (...) {
  }
  try {
    publishConfig.createMagnitudes =
        app->configGetBool("magnitudes.createMagnitudes");
  } catch (...) {
  }

  try {
    streamConfig.templateConfig.phase = app->configGetString("template.phase");
  } catch (...) {
  }
  try {
    streamConfig.templateConfig.wfStart =
        app->configGetDouble("template.waveformStart");
  } catch (...) {
  }
  try {
    streamConfig.templateConfig.wfEnd =
        app->configGetDouble("template.waveformEnd");
  } catch (...) {
  }

  try {
    streamConfig.filter = app->configGetString("processing.filter");
  } catch (...) {
  }
  try {
    streamConfig.initTime = app->configGetDouble("processing.initTime");
  } catch (...) {
  }
  try {
    detectorConfig.gapInterpolation =
        app->configGetBool("processing.gapInterpolation");
  } catch (...) {
  }
  try {
    detectorConfig.gapThreshold =
        app->configGetDouble("processing.minGapLength");
  } catch (...) {
  }
  try {
    detectorConfig.gapTolerance =
        app->configGetDouble("processing.maxGapLength");
  } catch (...) {
  }

  try {
    detectorConfig.triggerOn =
        app->configGetDouble("detector.triggerOnThreshold");
  } catch (...) {
  }
  try {
    detectorConfig.triggerOff =
        app->configGetDouble("detector.triggerOffThreshold");
  } catch (...) {
  }
  try {
    detectorConfig.triggerDuration =
        app->configGetDouble("detector.triggerDuration");
  } catch (...) {
  }
  try {
    detectorConfig.timeCorrection =
        app->configGetDouble("detector.timeCorrection");
  } catch (...) {
  }
  try {
    detectorConfig.arrivalOffsetThreshold =
        app->configGetDouble("detector.arrivalOffsetThreshold");
  } catch (...) {
  }
  try {
    detectorConfig.minArrivals = app->configGetInt("detector.minimumArrivals");
  } catch (...) {
  }
  try {
    detectorConfig.mergingStrategy =
        app->configGetString("detector.mergingStrategy");
  } catch (...) {
  }

  try {
    sensorLocationBindings.amplitudeProcessingConfig.mlx.filter =
        app->configGetString("amplitudes.filter");
  } catch (ValueException &e) {
    throw;
  } catch (...) {
  }
  try {
    sensorLocationBindings.amplitudeProcessingConfig.mlx.initTime =
        app->configGetDouble("amplitudes.initTime");
  } catch (ValueException &e) {
    throw;
  } catch (...) {
  }
}

void Application::Config::init(const System::CommandLine &commandline) {
  templatesPrepare = commandline.hasOption("templates-prepare");
  templatesNoCache = commandline.hasOption("templates-reload");

  if (commandline.hasOption("templates-json")) {
    Environment *env{Environment::Instance()};
    pathTemplateJson =
        env->absolutePath(commandline.option<std::string>("templates-json"));
  }

  playbackConfig.enabled = commandline.hasOption("playback");

  offlineMode = commandline.hasOption("offline");
  noPublish = commandline.hasOption("no-publish");
}

}  // namespace detect
}  // namespace Seiscomp
