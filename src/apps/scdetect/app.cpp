#include "app.h"

#include <algorithm>
#include <ios>
#include <unordered_set>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <seiscomp/core/arrayfactory.h>
#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/arrival.h>
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

#include "builder.h"
#include "config.h"
#include "detector.h"
#include "detector/arrival.h"
#include "eventstore.h"
#include "log.h"
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

} // namespace

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

const char *Application::version() { return kVersion; }

void Application::createCommandLineDescription() {
  StreamApplication::createCommandLineDescription();

  commandline().addOption(
      "Database", "event-db",
      "load events from the given database or file, format: "
      "[service://]location",
      &config_.url_event_db);

  commandline().addOption(
      "Records", "record-starttime",
      "defines a start time (YYYY-MM-DDTHH:MM:SS formatted) for "
      "requesting records from the configured archive recordstream; "
      "implicitly enables reprocessing/playback mode",
      &config_.playback_config.start_time_str);
  commandline().addOption(
      "Records", "record-endtime",
      "defines an end time (YYYY-MM-DDTHH:MM:SS formatted) for "
      "requesting records from the configured archive recordstream; "
      "implicitly enables reprocessing/playback mode",
      &config_.playback_config.end_time_str);

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
      &config_.path_ep);
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
      &config_.path_template_json);
}

bool Application::validateParameters() {
  if (!StreamApplication::validateParameters())
    return false;

  config_.Init(commandline());

  // TODO(damb): Disable messaging (offline mode) with certain command line
  // options
  if (config_.offline_mode) {
    SCDETECT_LOG_INFO("Disable messaging");
    setMessagingEnabled(false);

    config_.no_publish = true;
  }

  if (!config_.no_publish && commandline().hasOption("ep")) {
    config_.no_publish = true;
  }

  // disable the database if required
  if (!isInventoryDatabaseEnabled()) {
    SCDETECT_LOG_INFO("Disable database connection");
    setDatabaseEnabled(false, false);
  }

  // validate paths
  if (!config_.path_template_json.empty() &&
      !Util::fileExists(config_.path_template_json)) {
    SCDETECT_LOG_ERROR("Invalid path to template configuration file: %s",
                       config_.path_template_json.c_str());
    return false;
  }

  // validate reprocessing config
  auto ValidateAndStoreTime = [](const std::string &time_str,
                                 Core::Time &result) {
    if (!time_str.empty() && !result.fromString(time_str.c_str(), "%FT%T")) {

      SCDETECT_LOG_ERROR("Invalid time: %s", time_str.c_str());
      return false;
    }
    return true;
  };

  if (!ValidateAndStoreTime(config_.playback_config.start_time_str,
                            config_.playback_config.start_time)) {
    return false;
  }
  if (!ValidateAndStoreTime(config_.playback_config.end_time_str,
                            config_.playback_config.end_time)) {
    return false;
  }

  if (!config::ValidateXCorrThreshold(config_.detector_config.trigger_on)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'triggerOnThreshold': %f. Not in "
        "interval [-1,1].",
        config_.detector_config.trigger_on);
    return false;
  }
  if (!config::ValidateXCorrThreshold(config_.detector_config.trigger_off)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'triggerOffThreshold': %f. Not in "
        "interval [-1,1].",
        config_.detector_config.trigger_off);
    return false;
  }

  if (!utils::IsGeZero(config_.stream_config.init_time)) {
    SCDETECT_LOG_ERROR("Invalid configuration: 'initTime': %f. Must be "
                       "greater equal 0.",
                       config_.stream_config.init_time);
    return false;
  }

  if (!config::ValidateArrivalOffsetThreshold(
          config_.detector_config.arrival_offset_threshold)) {
    SCDETECT_LOG_ERROR("Invalid configuration: 'arrivalOffsetThreshold': %f. "
                       "Must be < 0 or >= 2.0e-6",
                       config_.detector_config.arrival_offset_threshold);
    return false;
  }

  if (!config::ValidateMinArrivals(config_.detector_config.min_arrivals)) {
    SCDETECT_LOG_ERROR("Invalid configuration: 'minimumArrivals': %d. "
                       "Must be < 0 or >= 1",
                       config_.detector_config.min_arrivals);
    return false;
  }

  if (config_.stream_config.template_config.wf_start >=
      config_.stream_config.template_config.wf_end) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'waveformStart' >= 'waveformEnd' : %f >= %f",
        config_.stream_config.template_config.wf_start,
        config_.stream_config.template_config.wf_end);
    return false;
  }

  return true;
}

bool Application::initConfiguration() {

  if (!StreamApplication::initConfiguration())
    return false;

  config_.Init(this);

  return true;
}

bool Application::init() {

  if (!StreamApplication::init())
    return false;

  if (config_.playback_config.enabled) {
    SCDETECT_LOG_INFO("Playback mode enabled");
  }

  // TODO(damb): Check if std::unique_ptr wouldn't be sufficient, here.
  WaveformHandlerIfacePtr waveform_handler{
      utils::make_smart<WaveformHandler>(recordStreamURL())};
  if (!config_.templates_no_cache) {
    // cache template waveforms on filesystem
    waveform_handler = utils::make_smart<FileSystemCache>(
        waveform_handler, config_.path_filesystem_cache,
        settings::kCacheRawWaveforms);
  }

  if (!InitDetectors(waveform_handler))
    return false;

  output_origins_ = addOutputObjectLog("origin", primaryMessagingGroup());

  return true;
}

bool Application::run() {
  SCDETECT_LOG_DEBUG("Application initialized.");

  if (config_.templates_prepare) {
    SCDETECT_LOG_DEBUG(
        "Requested application exit after template initialization.");
    return true;
  }

  if (commandline().hasOption("ep")) {
    ep_ = utils::make_smart<DataModel::EventParameters>();
  }

  // subscribe to streams
  for (const auto &detector_pair : detectors_) {
    utils::WaveformStreamID wf_stream_id{detector_pair.first};

    recordStream()->addStream(wf_stream_id.net_code(), wf_stream_id.sta_code(),
                              wf_stream_id.loc_code(), wf_stream_id.cha_code());
  }

  if (!config_.playback_config.start_time_str.empty()) {
    recordStream()->setStartTime(config_.playback_config.start_time);
    config_.playback_config.enabled = true;
  }
  if (!config_.playback_config.end_time_str.empty()) {
    recordStream()->setEndTime(config_.playback_config.end_time);
    config_.playback_config.enabled = true;
  }

  return StreamApplication::run();
}

void Application::done() {
  if (!config_.templates_prepare) {
    std::unordered_set<std::string> detector_ids;
    // terminate detectors
    for (const auto &detector_pair : detectors_) {
      auto &detector{detector_pair.second};
      const auto detector_id{detector->id()};
      if (detector_ids.find(detector_id) == detector_ids.end()) {
        detector_ids.emplace(detector->id());
        detector->Terminate();
      }
    }

    if (ep_) {
      IO::XMLArchive ar;
      ar.create(config_.path_ep.empty() ? "-" : config_.path_ep.c_str());
      ar.setFormattedOutput(true);
      ar << ep_;
      ar.close();
      SCDETECT_LOG_DEBUG("Found %lu origins.", ep_->originCount());
      ep_.reset();
    }
  }

  EventStore::Instance().Reset();

  StreamApplication::done();
}

void Application::handleRecord(Record *rec) {
  if (!rec->data())
    return;

  auto range{detectors_.equal_range(std::string{rec->streamID()})};
  for (auto it = range.first; it != range.second; ++it) {

    auto &detector_ptr{it->second};
    if (detector_ptr->enabled()) {
      if (!detector_ptr->Feed(rec)) {
        SCDETECT_LOG_WARNING(
            "%s: Failed to feed record into detector (id=%s). Resetting.",
            it->first.c_str(), detector_ptr->id().c_str());
        detector_ptr->Reset();
        continue;
      }

      if (detector_ptr->finished()) {
        SCDETECT_LOG_WARNING("%s: Detector (id=%s) finished (status=%d, "
                             "status_value=%f). Resetting.",
                             it->first.c_str(), detector_ptr->id().c_str(),
                             utils::as_integer(detector_ptr->status()),
                             detector_ptr->status_value());
        detector_ptr->Reset();
        continue;
      }
    } else {
      SCDETECT_LOG_DEBUG(
          "%s: Skip feeding record into detector (id=%s). Reason: Disabled.",
          it->first.c_str(), detector_ptr->id().c_str());
    }
  }
}

void Application::EmitDetection(const WaveformProcessor *processor,
                                const Record *record,
                                const WaveformProcessor::ResultCPtr &result) {

  const auto detection{
      boost::dynamic_pointer_cast<const Detector::Detection>(result)};

  SCDETECT_LOG_DEBUG_TAGGED(
      processor->id(), "Creating origin (time=%s, detected_arrivals=%d) ...",
      detection->time.iso().c_str(), detection->template_results.size());
  Core::Time now{Core::Time::GMT()};

  DataModel::CreationInfo ci;
  ci.setAgencyID(agencyID());
  ci.setAuthor(author());
  ci.setCreationTime(now);

  DataModel::MagnitudePtr magnitude{DataModel::Magnitude::Create()};
  magnitude->setCreationInfo(ci);
  magnitude->setType(settings::kMagnitudeType);
  magnitude->setMagnitude(DataModel::RealQuantity(detection->magnitude));
  magnitude->setStationCount(detection->num_stations_used);

  DataModel::OriginPtr origin{DataModel::Origin::Create()};
  origin->setCreationInfo(ci);
  origin->setLatitude(DataModel::RealQuantity(detection->latitude));
  origin->setLongitude(DataModel::RealQuantity(detection->longitude));
  origin->setDepth(DataModel::RealQuantity(detection->depth));
  origin->setTime(DataModel::TimeQuantity(detection->time));
  origin->setEpicenterFixed(true);
  origin->setEvaluationMode(DataModel::EvaluationMode(DataModel::AUTOMATIC));

  std::vector<double> azimuths;
  std::vector<double> distances;
  for (const auto &result_pair : detection->template_results) {
    double az, baz, dist;
    const auto &sensor_location{result_pair.second.sensor_location};
    Math::Geo::delazi(detection->latitude, detection->longitude,
                      sensor_location.latitude, sensor_location.longitude,
                      &dist, &az, &baz);

    distances.push_back(dist);
    azimuths.push_back(az);
  }

  std::sort(azimuths.begin(), azimuths.end());
  std::sort(distances.begin(), distances.end());

  DataModel::OriginQuality origin_quality;
  if (azimuths.size() > 2) {
    double az_gap{};
    for (size_t i = 0; i < azimuths.size() - 1; ++i)
      az_gap = (azimuths[i + 1] - azimuths[i]) > az_gap
                   ? (azimuths[i + 1] - azimuths[i])
                   : az_gap;

    origin_quality.setAzimuthalGap(az_gap);
    magnitude->setAzimuthalGap(az_gap);
  }

  if (!distances.empty()) {
    origin_quality.setMinimumDistance(distances.front());
    origin_quality.setMaximumDistance(distances.back());
    origin_quality.setMedianDistance(distances[distances.size() / 2]);
  }

  origin_quality.setStandardError(1.0 - detection->fit);
  origin_quality.setAssociatedStationCount(detection->num_stations_associated);
  origin_quality.setUsedStationCount(detection->num_stations_used);
  origin_quality.setAssociatedPhaseCount(detection->num_channels_associated);
  origin_quality.setUsedPhaseCount(detection->num_channels_used);

  origin->setQuality(origin_quality);
  origin->setMethodID(settings::kOriginMethod);

  origin->setQuality(origin_quality);
  origin->setMethodID(settings::kOriginMethod);

  const auto CreatePick = [](const detector::Arrival &a) {
    DataModel::PickPtr ret{DataModel::Pick::Create()};

    ret->setTime(DataModel::TimeQuantity{a.pick.time, boost::none,
                                         a.pick.lower_uncertainty,
                                         a.pick.upper_uncertainty});
    utils::WaveformStreamID wf_id{a.pick.waveform_id};
    ret->setWaveformID(
        DataModel::WaveformStreamID{wf_id.net_code(), wf_id.sta_code(),
                                    wf_id.loc_code(), wf_id.cha_code(), ""});
    ret->setEvaluationMode(DataModel::EvaluationMode(DataModel::AUTOMATIC));

    if (a.pick.phase_hint) {
      ret->setPhaseHint(DataModel::Phase{*a.pick.phase_hint});
    }
    return ret;
  };

  const auto CreateArrival = [&ci](const detector::Arrival &a,
                                   const DataModel::PickCPtr &pick) {
    auto ret{utils::make_smart<DataModel::Arrival>()};
    ret->setCreationInfo(ci);
    ret->setPickID(pick->publicID());
    ret->setPhase(a.phase);
    if (a.weight) {
      ret->setWeight(a.weight);
    }
    return ret;
  };

  std::vector<ArrivalPick> arrival_picks;
  if (detection->with_arrivals) {
    for (const auto &result_pair : detection->template_results) {
      const auto &res{result_pair.second};

      const auto pick{CreatePick(res.arrival)};
      const auto arrival{CreateArrival(res.arrival, pick)};
      arrival_picks.push_back({arrival, pick});
    }
  }

  // create theoretical template arrivals
  for (const auto &a : detection->theoretical_template_arrivals) {
    const auto pick{CreatePick(a)};
    const auto arrival{CreateArrival(a, pick)};
    arrival_picks.push_back({arrival, pick});
  }

  // TODO(damb): Attach StationMagnitudeContribution related stuff.

  logObject(output_origins_, Core::Time::GMT());

  if (connection() && !config_.no_publish) {
    SCDETECT_LOG_DEBUG_TAGGED(processor->id(), "Sending event parameters ...");

    auto notifier_msg{utils::make_smart<DataModel::NotifierMessage>()};

    // origin
    auto notifier{utils::make_smart<DataModel::Notifier>(
        "EventParameters", DataModel::OP_ADD, origin.get())};
    notifier_msg->attach(notifier.get());

    // magnitude
    {
      auto notifier{utils::make_smart<DataModel::Notifier>(
          origin->publicID(), DataModel::OP_ADD, magnitude.get())};
      notifier_msg->attach(notifier.get());
    }

    for (auto &arrival_pick : arrival_picks) {
      // pick
      {
        auto notifier{utils::make_smart<DataModel::Notifier>(
            "EventParameters", DataModel::OP_ADD, arrival_pick.pick.get())};

        notifier_msg->attach(notifier.get());
      }
      // arrival
      {
        auto notifier{utils::make_smart<DataModel::Notifier>(
            origin->publicID(), DataModel::OP_ADD, arrival_pick.arrival.get())};

        notifier_msg->attach(notifier.get());
      }
    }

    if (!connection()->send(notifier_msg.get())) {
      SCDETECT_LOG_ERROR_TAGGED(processor->id(),
                                "Sending of event parameters failed.");
    }
  }

  if (ep_) {
    ep_->add(origin.get());

    origin->add(magnitude.get());

    for (auto &arrival_pick : arrival_picks) {
      origin->add(arrival_pick.arrival.get());

      ep_->add(arrival_pick.pick.get());
    }
  }
}

bool Application::LoadEvents(const std::string &event_db,
                             DataModel::DatabaseQueryPtr db) {
  bool loaded{false};
  if (!event_db.empty()) {
    SCDETECT_LOG_INFO("Loading events from %s", event_db.c_str());
    if (event_db.find("://") == std::string::npos) {
      try {
        EventStore::Instance().Load(event_db);
        loaded = true;
      } catch (std::exception &e) {
        SCDETECT_LOG_ERROR("%s", e.what());
      }
    } else if (event_db.find("file://") == 0) {
      try {
        EventStore::Instance().Load(event_db.substr(7));
        loaded = true;
      } catch (std::exception &e) {
        SCDETECT_LOG_ERROR("%s", e.what());
      }
    } else {
      SCDETECT_LOG_INFO("Trying to connect to %s", event_db.c_str());
      IO::DatabaseInterfacePtr db{
          IO::DatabaseInterface::Open(event_db.c_str())};
      if (db) {
        SCDETECT_LOG_INFO("Connected successfully");
        auto query{utils::make_smart<DataModel::DatabaseQuery>(db.get())};
        EventStore::Instance().Load(query);
        loaded = true;
      } else {
        SCDETECT_LOG_WARNING("Database connection to %s failed",
                             event_db.c_str());
      }
    }
  }

  if (!loaded && isDatabaseEnabled()) {
    SCDETECT_LOG_INFO("Loading events from %s", databaseURI().c_str());
    try {
      EventStore::Instance().Load(query());
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

bool Application::InitDetectors(WaveformHandlerIfacePtr waveform_handler) {

  // load event related data
  if (!LoadEvents(config_.url_event_db, query())) {
    SCDETECT_LOG_ERROR("Failed to load events.");
    return false;
  }

  if (!EventStore::Instance().event_parameters()) {
    SCDETECT_LOG_ERROR("No event parameters found.");
    return false;
  }

  config_.path_filesystem_cache =
      boost::filesystem::path(config_.path_filesystem_cache).string();
  if (!Util::pathExists(config_.path_filesystem_cache) &&
      !Util::createPath(config_.path_filesystem_cache)) {
    SCDETECT_LOG_ERROR("Failed to create path: %s",
                       config_.path_filesystem_cache.c_str());
    return false;
  }

  // load template related data
  // TODO(damb): Allow parsing template configuration from profiles
  if (config_.path_template_json.empty()) {
    SCDETECT_LOG_ERROR("Missing template configuration file.");
    return false;
  }

  // initialize detectors
  std::unordered_set<std::string> processor_ids;
  auto IsUniqueProcessorId = [&processor_ids](const std::string &id) {
    bool id_exists{processor_ids.find(id) != processor_ids.end()};
    if (id_exists) {
      SCDETECT_LOG_WARNING("Processor id is be used by multiple processors: %s",
                           id.c_str());
    }
    processor_ids.emplace(id);
    return !id_exists;
  };

  try {
    SCDETECT_LOG_INFO("Loading template configuration from %s",
                      config_.path_template_json.c_str());

    std::ifstream ifs{config_.path_template_json};
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ifs, pt);

    for (const auto &template_setting_pt : pt) {
      try {
        TemplateConfig tc{template_setting_pt.second, config_.detector_config,
                          config_.stream_config};

        SCDETECT_LOG_DEBUG("Creating detector processor (id=%s) ... ",
                           tc.detector_id().c_str());

        auto detector_builder{
            std::move(Detector::Create(tc.detector_id(), tc.origin_id())
                          .set_config(tc.detector_config(),
                                      config_.playback_config.enabled)
                          .set_eventparameters())};

        std::vector<std::string> stream_ids;
        for (const auto &stream_config_pair : tc) {
          IsUniqueProcessorId(stream_config_pair.second.template_id);
          try {
            detector_builder.set_stream(stream_config_pair.first,
                                        stream_config_pair.second,
                                        waveform_handler);
          } catch (builder::NoSensorLocation &e) {
            if (config_.skip_template_if_no_sensor_location_data) {
              SCDETECT_LOG_WARNING(
                  "%s (%s): No sensor location data for template processor "
                  "available. Skipping.",
                  stream_config_pair.first.c_str(),
                  stream_config_pair.second.template_config.wf_stream_id
                      .c_str());
              continue;
            }
            throw;
          } catch (builder::NoStream &e) {
            if (config_.skip_template_if_no_stream_data) {
              SCDETECT_LOG_WARNING(
                  "%s (%s): No stream data for template processor "
                  "available. Skipping.",
                  stream_config_pair.first.c_str(),
                  stream_config_pair.second.template_config.wf_stream_id
                      .c_str());
              continue;
            }
            throw;
          } catch (builder::NoWaveformData &e) {
            if (config_.skip_template_if_no_waveform_data) {
              SCDETECT_LOG_WARNING(
                  "%s (%s): No waveform data for template processor "
                  "available. Skipping.",
                  stream_config_pair.first.c_str(),
                  stream_config_pair.second.template_config.wf_stream_id
                      .c_str());
              continue;
            }
            throw;
          }
          stream_ids.push_back(stream_config_pair.first);
        }

        std::shared_ptr<detect::Detector> detector_ptr{
            detector_builder.Build()};
        detector_ptr->set_result_callback(
            [this](const WaveformProcessor *proc, const Record *rec,
                   const WaveformProcessor::ResultCPtr &res) {
              EmitDetection(proc, rec, res);
            });
        for (const auto &stream_id : stream_ids)
          detectors_.emplace(stream_id, detector_ptr);

      } catch (Exception &e) {
        SCDETECT_LOG_WARNING("Failed to create detector: %s. Skipping.",
                             e.what());
        continue;
      }
    }
  } catch (boost::property_tree::json_parser::json_parser_error &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        config_.path_template_json.c_str(), e.what());
    return false;
  } catch (std::ifstream::failure &e) {
    SCDETECT_LOG_ERROR(
        "Failed to parse JSON template configuration file (%s): %s",
        config_.path_template_json.c_str(), e.what());
    return false;
  }
  return true;
}

Application::Config::Config() {
  Environment *env{Environment::Instance()};

  boost::filesystem::path sc_install_dir{env->installDir()};
  boost::filesystem::path path_cache{sc_install_dir /
                                     settings::kPathFilesystemCache};
  path_filesystem_cache = path_cache.string();
}

void Application::Config::Init(const Client::Application *app) {
  try {
    stream_config.template_config.phase =
        app->configGetString("template.phase");
  } catch (...) {
  }
  try {
    stream_config.template_config.wf_start =
        app->configGetDouble("template.waveformStart");
  } catch (...) {
  }
  try {
    stream_config.template_config.wf_end =
        app->configGetDouble("template.waveformEnd");
  } catch (...) {
  }

  try {
    stream_config.init_time = app->configGetDouble("processing.initTime");
  } catch (...) {
  }
  try {
    detector_config.gap_interpolation =
        app->configGetBool("processing.gapInterpolation");
  } catch (...) {
  }
  try {
    detector_config.gap_threshold =
        app->configGetDouble("processing.minGapLength");
  } catch (...) {
  }
  try {
    detector_config.gap_tolerance =
        app->configGetDouble("processing.maxGapLength");
  } catch (...) {
  }

  try {
    detector_config.trigger_on =
        app->configGetDouble("detector.triggerOnThreshold");
  } catch (...) {
  }
  try {
    detector_config.trigger_off =
        app->configGetDouble("detector.triggerOffThreshold");
  } catch (...) {
  }
  try {
    detector_config.trigger_duration =
        app->configGetDouble("detector.triggerDuration");
  } catch (...) {
  }
  try {
    detector_config.time_correction =
        app->configGetDouble("detector.timeCorrection");
  } catch (...) {
  }
  try {
    detector_config.create_arrivals =
        app->configGetBool("detector.createArrivals");
  } catch (...) {
  }
  try {
    detector_config.create_template_arrivals =
        app->configGetBool("detector.createTemplateArrivals");
  } catch (...) {
  }
  try {
    detector_config.arrival_offset_threshold =
        app->configGetDouble("detector.arrivalOffsetThreshold");
  } catch (...) {
  }
  try {
    detector_config.min_arrivals =
        app->configGetInt("detector.minimumArrivals");
  } catch (...) {
  }
}

void Application::Config::Init(const System::CommandLine &commandline) {
  templates_prepare = commandline.hasOption("templates-prepare");
  templates_no_cache = commandline.hasOption("templates-reload");

  playback_config.enabled = commandline.hasOption("playback");

  offline_mode = commandline.hasOption("offline");
  no_publish = commandline.hasOption("no-publish");
}

} // namespace detect
} // namespace Seiscomp
