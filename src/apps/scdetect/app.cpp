#include "app.h"

#include <seiscomp/core/arrayfactory.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/strings.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/comment.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/notifier.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/phase.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/timequantity.h>
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/io/recordinput.h>
#include <seiscomp/math/geo.h>
#include <seiscomp/utils/files.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/smart_ptr/intrusive_ptr.hpp>
#include <ios>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "builder.h"
#include "config.h"
#include "detector/arrival.h"
#include "detector/detectorbuilder.h"
#include "detector/detectorwaveformprocessor.h"
#include "eventstore.h"
#include "log.h"
#include "resamplerstore.h"
#include "settings.h"
#include "utils.h"
#include "validators.h"
#include "version.h"

namespace Seiscomp {
namespace detect {

namespace {

struct ArrivalPick {
  DataModel::ArrivalPtr arrival;
  DataModel::PickPtr pick;
};

}  // namespace

Application::Application(int argc, char **argv)
    : StreamApplication(argc, argv) {
  setLoadStationsEnabled(true);
  setLoadInventoryEnabled(true);
  setLoadConfigModuleEnabled(true);
  setMessagingEnabled(true);

  setPrimaryMessagingGroup("LOCATION");
}

Application::~Application() {}

Application::BaseException::BaseException()
    : Exception{"base application exception"} {}

Application::ConfigError::ConfigError()
    : BaseException{"application configuration error"} {}

Application::DuplicatePublicObjectId::DuplicatePublicObjectId()
    : BaseException{"dubplicate public object identifier"} {}

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

  commandline().addGroup("Input");
  commandline().addOption(
      "Input", "templates-json",
      "path to a template configuration file (json-formatted)",
      &_config.pathTemplateJson);
}

bool Application::validateParameters() {
  if (!StreamApplication::validateParameters()) return false;

  _config.init(commandline());

  // TODO(damb): Disable messaging (offline mode) with certain command line
  // options
  if (_config.offlineMode) {
    SCDETECT_LOG_INFO("Disable messaging");
    setMessagingEnabled(false);

    _config.noPublish = true;
  }

  if (!_config.noPublish && commandline().hasOption("ep")) {
    _config.noPublish = true;
  }

  // disable the database if required
  if (!isInventoryDatabaseEnabled()) {
    SCDETECT_LOG_INFO("Disable database connection");
    setDatabaseEnabled(false, false);
  }

  // validate paths
  if (!_config.pathTemplateJson.empty() &&
      !Util::fileExists(_config.pathTemplateJson)) {
    SCDETECT_LOG_ERROR("Invalid path to template configuration file: %s",
                       _config.pathTemplateJson.c_str());
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

  if (!utils::isGeZero(_config.streamConfig.initTime)) {
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

bool Application::initConfiguration() {
  if (!StreamApplication::initConfiguration()) return false;

  _config.init(this);

  return true;
}

bool Application::init() {
  if (!StreamApplication::init()) return false;

  _outputOrigins = addOutputObjectLog("origin", primaryMessagingGroup());

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
      utils::make_smart<WaveformHandler>(recordStreamURL())};
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

    waveformHandler = utils::make_smart<FileSystemCache>(
        waveformHandler, _config.pathFilesystemCache,
        settings::kCacheRawWaveforms);
  }
  // cache demeaned template waveform snippets in order to speed up the
  // initialization procedure
  waveformHandler =
      utils::make_smart<InMemoryCache>(waveformHandler, /*raw=*/false);

  // load template related data
  // TODO(damb):
  //
  // - Allow parsing template configuration from profiles
  if (_config.pathTemplateJson.empty()) {
    SCDETECT_LOG_ERROR("Missing template configuration file.");
    return false;
  }

  try {
    std::ifstream ifs{_config.pathTemplateJson};
    if (!initDetectors(ifs, waveformHandler.get())) {
      return false;
    }
  } catch (std::ifstream::failure &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        _config.pathTemplateJson.c_str(), e.what());
    return false;
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

  if (commandline().hasOption("ep")) {
    _ep = utils::make_smart<DataModel::EventParameters>();
  }

  // subscribe to streams
  for (const auto &detectorPair : _detectors) {
    utils::WaveformStreamID wfStreamId{detectorPair.first};

    recordStream()->addStream(wfStreamId.netCode(), wfStreamId.staCode(),
                              wfStreamId.locCode(), wfStreamId.chaCode());
  }

  if (!_config.playbackConfig.startTimeStr.empty()) {
    recordStream()->setStartTime(_config.playbackConfig.startTime);
    _config.playbackConfig.enabled = true;
  }
  if (!_config.playbackConfig.endTimeStr.empty()) {
    recordStream()->setEndTime(_config.playbackConfig.endTime);
    _config.playbackConfig.enabled = true;
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

  auto range{_detectors.equal_range(std::string{rec->streamID()})};
  for (auto it = range.first; it != range.second; ++it) {
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
            utils::asInteger(detector->status()), detector->statusValue());
        detector->reset();
        continue;
      }
    } else {
      SCDETECT_LOG_DEBUG(
          "%s: Skip feeding record into detector (id=%s). Reason: Disabled.",
          it->first.c_str(), detector->id().c_str());
    }
  }
}

void Application::emitDetection(
    const detector::DetectorWaveformProcessor *processor, const Record *record,
    const detector::DetectorWaveformProcessor::DetectionCPtr &detection) {
  SCDETECT_LOG_DEBUG_PROCESSOR(
      processor, "Creating origin (time=%s, detected_arrivals=%d) ...",
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
    auto comment{utils::make_smart<DataModel::Comment>()};
    comment->setId("scdetectDetectorId");
    comment->setText(processor->id());
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

  const auto createPick = [](const detector::Arrival &arrival) {
    DataModel::PickPtr ret{DataModel::Pick::Create()};
    if (!ret) {
      throw DuplicatePublicObjectId{"duplicate pick identifier"};
    }

    ret->setTime(DataModel::TimeQuantity{arrival.pick.time, boost::none,
                                         arrival.pick.lowerUncertainty,
                                         arrival.pick.upperUncertainty});
    utils::WaveformStreamID wfStreamId{arrival.pick.waveformStreamId};
    ret->setWaveformID(DataModel::WaveformStreamID{
        wfStreamId.netCode(), wfStreamId.staCode(), wfStreamId.locCode(),
        wfStreamId.chaCode(), ""});
    ret->setEvaluationMode(DataModel::EvaluationMode(DataModel::AUTOMATIC));

    if (arrival.pick.phaseHint) {
      ret->setPhaseHint(DataModel::Phase{*arrival.pick.phaseHint});
    }
    return ret;
  };

  const auto createArrival = [&ci](const detector::Arrival &arrival,
                                   const DataModel::PickCPtr &pick) {
    auto ret{utils::make_smart<DataModel::Arrival>()};
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

  std::vector<ArrivalPick> arrivalPicks;
  if (detection->publishConfig.createArrivals) {
    for (const auto &resultPair : detection->templateResults) {
      const auto &res{resultPair.second};

      try {
        const auto pick{createPick(res.arrival)};
        {
          auto comment{utils::make_smart<DataModel::Comment>()};
          comment->setId(settings::kTemplateWaveformDurationPickCommentId);
          comment->setText(
              Core::stringify("%lu.%lu", res.templateWaveformDuration.seconds(),
                              res.templateWaveformDuration.microseconds()));
          pick->add(comment.get());
        }

        const auto arrival{createArrival(res.arrival, pick)};
        arrivalPicks.push_back({arrival, pick});
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
        const auto pick{createPick(a)};
        const auto arrival{createArrival(a, pick)};
        arrivalPicks.push_back({arrival, pick});
      } catch (DuplicatePublicObjectId &e) {
        SCDETECT_LOG_WARNING_PROCESSOR(processor, "Internal error: %s",
                                       e.what());
        continue;
      }
    }
  }

  logObject(_outputOrigins, Core::Time::GMT());

  if (connection() && !_config.noPublish) {
    SCDETECT_LOG_DEBUG_PROCESSOR(processor, "Sending event parameters ...");

    auto notifierMsg{utils::make_smart<DataModel::NotifierMessage>()};

    // origin
    auto notifier{utils::make_smart<DataModel::Notifier>(
        "EventParameters", DataModel::OP_ADD, origin.get())};
    notifierMsg->attach(notifier.get());

    for (auto &arrivalPick : arrivalPicks) {
      // pick
      {
        auto notifier{utils::make_smart<DataModel::Notifier>(
            "EventParameters", DataModel::OP_ADD, arrivalPick.pick.get())};

        notifierMsg->attach(notifier.get());
      }
      // arrival
      {
        auto notifier{utils::make_smart<DataModel::Notifier>(
            origin->publicID(), DataModel::OP_ADD, arrivalPick.arrival.get())};

        notifierMsg->attach(notifier.get());
      }
    }

    if (!connection()->send(notifierMsg.get())) {
      SCDETECT_LOG_ERROR_PROCESSOR(processor,
                                   "Sending of event parameters failed.");
    }
  }

  if (_ep) {
    _ep->add(origin.get());

    for (auto &arrivalPick : arrivalPicks) {
      origin->add(arrivalPick.arrival.get());

      _ep->add(arrivalPick.pick.get());
    }
  }
}

bool Application::loadEvents(const std::string &eventDb,
                             DataModel::DatabaseQueryPtr db) {
  bool loaded{false};
  if (!eventDb.empty()) {
    SCDETECT_LOG_INFO("Loading events from %s", eventDb.c_str());
    if (eventDb.find("://") == std::string::npos) {
      try {
        EventStore::Instance().load(eventDb);
        loaded = true;
      } catch (std::exception &e) {
        SCDETECT_LOG_ERROR("%s", e.what());
      }
    } else if (eventDb.find("file://") == 0) {
      try {
        EventStore::Instance().load(eventDb.substr(7));
        loaded = true;
      } catch (std::exception &e) {
        SCDETECT_LOG_ERROR("%s", e.what());
      }
    } else {
      SCDETECT_LOG_INFO("Trying to connect to %s", eventDb.c_str());
      IO::DatabaseInterfacePtr db{IO::DatabaseInterface::Open(eventDb.c_str())};
      if (db) {
        SCDETECT_LOG_INFO("Connected successfully");
        auto query{utils::make_smart<DataModel::DatabaseQuery>(db.get())};
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
      SCDETECT_LOG_ERROR("%s", e.what());
    }
  }

  if (loaded) {
    SCDETECT_LOG_INFO("Finished loading events");
  }

  return loaded;
}

bool Application::initDetectors(std::ifstream &ifs,
                                WaveformHandlerIface *waveformHandler) {
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
    SCDETECT_LOG_INFO("Loading template configuration from %s",
                      _config.pathTemplateJson.c_str());

    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ifs, pt);

    for (const auto &templateSettingPt : pt) {
      try {
        TemplateConfig tc{templateSettingPt.second, _config.detectorConfig,
                          _config.streamConfig, _config.publishConfig};

        SCDETECT_LOG_DEBUG("Creating detector processor (id=%s) ... ",
                           tc.detectorId().c_str());

        auto detectorBuilder{
            std::move(detector::DetectorWaveformProcessor::Create(
                          tc.detectorId(), tc.originId())
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
                  "%s (%s): No sensor location data for template processor "
                  "available. Skipping.",
                  streamConfigPair.first.c_str(),
                  streamConfigPair.second.templateConfig.wfStreamId.c_str());
              continue;
            }
            throw;
          } catch (builder::NoStream &e) {
            if (_config.skipTemplateIfNoStreamData) {
              SCDETECT_LOG_WARNING(
                  "%s (%s): No stream data for template processor "
                  "available. Skipping.",
                  streamConfigPair.first.c_str(),
                  streamConfigPair.second.templateConfig.wfStreamId.c_str());
              continue;
            }
            throw;
          } catch (builder::NoWaveformData &e) {
            if (_config.skipTemplateIfNoWaveformData) {
              SCDETECT_LOG_WARNING(
                  "%s (%s): No waveform data for template processor "
                  "available. Skipping.",
                  streamConfigPair.first.c_str(),
                  streamConfigPair.second.templateConfig.wfStreamId.c_str());
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
            [this](const WaveformProcessor *proc, const Record *rec,
                   const WaveformProcessor::ResultCPtr &res) {
              emitDetection(
                  dynamic_cast<const detector::DetectorWaveformProcessor *>(
                      proc),
                  rec,
                  boost::dynamic_pointer_cast<
                      const detector::DetectorWaveformProcessor::Detection>(
                      res));
            });
        for (const auto &streamId : streamIds)
          _detectors.emplace(streamId, detectorPtr);

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
  }
  return true;
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
