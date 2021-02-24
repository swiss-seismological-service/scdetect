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
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/io/recordinput.h>
#include <seiscomp/math/geo.h>
#include <seiscomp/utils/files.h>

#include "builder.h"
#include "config.h"
#include "detector.h"
#include "eventstore.h"
#include "log.h"
#include "settings.h"
#include "utils.h"
#include "version.h"

namespace Seiscomp {
namespace detect {

namespace {

struct ArrivalPick {
  DataModel::ArrivalPtr arrival;
  DataModel::PickPtr pick;
};

#define NEW_OPT(var, ...) AddOption(&var, __VA_ARGS__)
#define NEW_OPT_CLI(var, ...) AddOption(&var, nullptr, __VA_ARGS__)

template <typename T>
void readConfig(const Client::Application *, T *storage, const char *name) {}

template <typename T>
void readCLI(const System::CommandLine *, T *storage, const char *name) {}

#define IMPL_READ_CONFIG(T, method)                                            \
  template <>                                                                  \
  void readConfig<T>(const Client::Application *app, T *storage,               \
                     const char *name) {                                       \
    try {                                                                      \
      *storage = app->method(name);                                            \
    } catch (...) {                                                            \
    }                                                                          \
  }

template <typename T> struct OptionImpl : Application::Option {
  OptionImpl(T *var, const char *cfg_name, const char *cli_group = nullptr,
             const char *cli_param = nullptr, const char *cli_desc = nullptr,
             bool cli_default = false, bool cli_switch = false)
      : Application::Option(cfg_name, cli_group, cli_param, cli_desc,
                            cli_default, cli_switch),
        storage(var) {}

  void Bind(System::CommandLine *cli) {
    if (cli_param == nullptr)
      return;
    if (cli_group != nullptr)
      cli->addGroup(cli_group);
    if (cli_switch)
      cli->addOption(cli_group ? cli_group : "Generic", cli_param, cli_desc);
    else
      cli->addOption(cli_group ? cli_group : "Generic", cli_param, cli_desc,
                     storage, cli_default);
  }

  bool Get(System::CommandLine *cli) {
    if (cli_param == nullptr)
      return true;
    if (cli_switch)
      readCLI(cli, storage, cli_param);
    return true;
  }

  bool Get(const Client::Application *app) {
    readConfig(app, storage, cfg_name);
    return true;
  }

  void PrintStorage(std::ostream &os) { os << *storage; }

  T *storage;
};

template <typename T> struct OptionVecImpl : Application::Option {
  OptionVecImpl(std::vector<T> *var, const char *cfg_name,
                const char *cli_group = nullptr,
                const char *cli_param = nullptr, const char *cli_desc = nullptr,
                bool cli_default = false, bool cli_switch = false)
      : Application::Option(cfg_name, cli_group, cli_param, cli_desc,
                            cli_default, cli_switch),
        storage(var) {}

  void Bind(System::CommandLine *cli) {
    if (cli_param == nullptr)
      return;

    if (cli_group != nullptr)
      cli->addGroup(cli_group);

    cli->addOption(cli_group ? cli_group : "Generic", cli_param, cli_desc,
                   storage);
  }

  bool Get(System::CommandLine *cli) {
    if (cli_param == nullptr)
      return true;
    if (cli_switch)
      readCLI(cli, storage, cli_param);
    return true;
  }

  bool Get(const Client::Application *app) {
    readConfig(app, storage, cfg_name);
    return true;
  }

  void PrintStorage(std::ostream &os) {
    bool first = true;
    for (typename std::vector<T>::iterator it = storage->begin();
         it != storage->end(); ++it) {
      if (!first)
        os << ",";
      else
        first = false;
      os << *it;
    }
  }

  std::vector<T> *storage;
};

template <typename T>
Application::OptionPtr
Bind(T *var, const char *cfg_name, const char *cli_group = nullptr,
     const char *cli_param = nullptr, const char *cli_desc = nullptr,
     bool cli_default = false, bool cli_switch = false) {
  return utils::make_smart<OptionImpl<T>>(var, cfg_name, cli_group, cli_param,
                                          cli_desc, cli_default, cli_switch);
}

template <typename T>
Application::OptionPtr
Bind(std::vector<T> *var, const char *cfg_name, const char *cli_group = nullptr,
     const char *cli_param = nullptr, const char *cli_desc = nullptr,
     bool cli_default = false, bool cli_switch = false) {
  return utils::make_smart<OptionVecImpl<T>>(
      var, cfg_name, cli_group, cli_param, cli_desc, cli_default, cli_switch);
}

IMPL_READ_CONFIG(int, configGetInt)
IMPL_READ_CONFIG(std::vector<int>, configGetInts)
IMPL_READ_CONFIG(double, configGetDouble)
IMPL_READ_CONFIG(std::vector<double>, configGetDoubles)
IMPL_READ_CONFIG(bool, configGetBool)
IMPL_READ_CONFIG(std::vector<bool>, configGetBools)
IMPL_READ_CONFIG(std::string, configGetString)
IMPL_READ_CONFIG(std::vector<std::string>, configGetStrings)

} // namespace

Application::Application(int argc, char **argv)
    : StreamApplication(argc, argv) {

  setLoadStationsEnabled(true);
  setLoadInventoryEnabled(true);
  setLoadConfigModuleEnabled(true);
  setMessagingEnabled(true);

  setPrimaryMessagingGroup("LOCATION");

  SetupConfigurationOptions();
}

Application::~Application() {}

Application::BaseException::BaseException()
    : Exception{"base application exception"} {}

Application::ConfigError::ConfigError()
    : BaseException{"application configuration error"} {}

const char *Application::version() { return kVersion; }

void Application::createCommandLineDescription() {
  StreamApplication::createCommandLineDescription();

  for (auto it = options_.begin(); it != options_.end(); ++it)
    (*it)->Bind(&commandline());
}

bool Application::validateParameters() {
  Environment *env{Environment::Instance()};

  boost::filesystem::path sc_install_dir{env->installDir()};
  boost::filesystem::path path_cache{sc_install_dir /
                                     settings::kPathFilesystemCache};
  config_.path_filesystem_cache = path_cache.string();

  if (!StreamApplication::validateParameters())
    return false;

  for (auto it = options_.begin(); it != options_.end(); ++it)
    if (!(*it)->Get(&commandline()))
      return false;

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

  if (!utils::ValidateXCorrThreshold(config_.detector_config.trigger_on)) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'triggerOnThreshold': %f. Not in "
        "interval [-1,1].",
        config_.detector_config.trigger_on);
    return false;
  }
  if (!utils::ValidateXCorrThreshold(config_.detector_config.trigger_off)) {
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

  if (!config_.stream_config.filter.empty()) {
    std::string err;
    if (!utils::IsValidFilter(config_.stream_config.filter, err)) {
      SCDETECT_LOG_WARNING("Invalid configuration: 'filter': %s", err.c_str());
      return false;
    }
  }

  if (config_.stream_config.template_config.wf_start >=
      config_.stream_config.template_config.wf_end) {
    SCDETECT_LOG_ERROR(
        "Invalid configuration: 'waveformStart' >= 'waveformEnd' : %f >= %f",
        config_.stream_config.template_config.wf_start,
        config_.stream_config.template_config.wf_end);
    return false;
  }
  if (!config_.stream_config.template_config.filter.empty()) {
    std::string err;
    if (!utils::IsValidFilter(config_.stream_config.template_config.filter,
                              err)) {
      SCDETECT_LOG_WARNING("Invalid configuration: 'filter': %s", err.c_str());
      return false;
    }
  }

  return true;
}

bool Application::initConfiguration() {

  if (!StreamApplication::initConfiguration())
    return false;

  for (auto it = options_.begin(); it != options_.end(); ++it)
    if (!(*it)->Get(this))
      return false;

  return true;
}

void Application::AddOption(OptionPtr opt) { options_.push_back(opt); }

const Application::Options &Application::options() const { return options_; }

#define IMPL_ADD_OPTION(TYPE)                                                  \
  void Application::AddOption(TYPE *var, const char *cfg_name,                 \
                              const char *cli_group, const char *cli_param,    \
                              const char *cli_desc, bool cli_default,          \
                              bool cli_switch) {                               \
    AddOption(Bind(var, cfg_name, cli_group, cli_param, cli_desc, cli_default, \
                   cli_switch));                                               \
  }

IMPL_ADD_OPTION(int)
IMPL_ADD_OPTION(double)
IMPL_ADD_OPTION(bool)
IMPL_ADD_OPTION(std::string)

IMPL_ADD_OPTION(std::vector<int>)
IMPL_ADD_OPTION(std::vector<double>)
IMPL_ADD_OPTION(std::vector<bool>)
IMPL_ADD_OPTION(std::vector<std::string>)

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

  if (config_.load_templates_only) {
    SCDETECT_LOG_DEBUG(
        "Requested application exit after template initialization.");
    return true;
  }

  if (config_.dump_debug_info) {
    SCDETECT_LOG_DEBUG("Dumping debug info enabled.");
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
  std::unordered_set<std::string> detector_ids;
  // terminate detectors
  for (const auto &detector_pair : detectors_) {
    auto &detector{detector_pair.second};
    const auto detector_id{detector->id()};
    if (detector_ids.find(detector_id) == detector_ids.end()) {
      detector_ids.emplace(detector->id());
      detector->Terminate();

      // optionally, create debug info files
      if (config_.dump_debug_info) {
        const auto path_debug_info{detector->debug_info_dir()};
        const auto fpath{path_debug_info / settings::kFnameDebugInfo};

        if (!Util::pathExists(path_debug_info.string())) {
          if (!Util::createPath(path_debug_info.string())) {
            SCDETECT_LOG_WARNING("Failed to create directory: %s",
                                 path_debug_info.c_str());
            continue;
          }
        }

        std::ofstream ofs{fpath.string()};
        if (ofs.is_open()) {
          ofs << detector->DebugString();
          ofs.close();
        } else {
          SCDETECT_LOG_WARNING("Failed to create file: %s", fpath.c_str());
        }
      }
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

void Application::EmitDetection(const Processor *processor,
                                const Record *record,
                                const Processor::ResultCPtr &result) {

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

  std::vector<ArrivalPick> arrival_picks;
  auto with_picks{processor->WithPicks()};
  if (with_picks) {
    for (const auto &result_pair : detection->template_results) {
      const auto &res{result_pair.second};
      DataModel::PickPtr pick{DataModel::Pick::Create()};

      pick->setTime(res.arrival.pick.time);
      pick->setWaveformID(res.arrival.pick.waveform_id);
      pick->setEvaluationMode(DataModel::EvaluationMode(DataModel::AUTOMATIC));

      if (res.arrival.pick.phase_hint) {
        pick->setPhaseHint(DataModel::Phase{*res.arrival.pick.phase_hint});
      }

      // create arrival
      auto arrival{utils::make_smart<DataModel::Arrival>()};
      arrival->setCreationInfo(ci);
      arrival->setPickID(pick->publicID());
      arrival->setPhase(res.arrival.phase);
      if (res.arrival.weight)
        arrival->setWeight(res.arrival.weight);

      arrival_picks.push_back({arrival, pick});
    }
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

    if (with_picks) {
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
              origin->publicID(), DataModel::OP_ADD,
              arrival_pick.arrival.get())};

          notifier_msg->attach(notifier.get());
        }
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

    if (with_picks) {
      for (auto &arrival_pick : arrival_picks) {
        origin->add(arrival_pick.arrival.get());

        ep_->add(arrival_pick.pick.get());
      }
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

void Application::SetupConfigurationOptions() {
  // define application specific configuration
  NEW_OPT(config_.stream_config.template_config.phase, "template.phase");
  NEW_OPT(config_.stream_config.template_config.wf_start,
          "template.waveformStart");
  NEW_OPT(config_.stream_config.template_config.wf_end, "template.waveformEnd");
  NEW_OPT(config_.stream_config.template_config.filter, "template.filter");

  NEW_OPT(config_.stream_config.sensitivity_correction,
          "processing.sensitivityCorrection");
  NEW_OPT(config_.stream_config.init_time, "processing.initTime");
  NEW_OPT(config_.stream_config.filter, "processing.filter");
  NEW_OPT(config_.detector_config.gap_interpolation,
          "processing.gapInterpolation");

  NEW_OPT(config_.detector_config.gap_threshold, "processing.minGapLength");
  NEW_OPT(config_.detector_config.gap_tolerance, "processing.maxGapLength");
  NEW_OPT(config_.detector_config.trigger_on, "detector.triggerOnThreshold");
  NEW_OPT(config_.detector_config.trigger_off, "detector.triggerOffThreshold");
  NEW_OPT(config_.detector_config.trigger_duration, "detector.triggerDuration");
  NEW_OPT(config_.detector_config.time_correction, "detector.timeCorrection");
  NEW_OPT(config_.detector_config.create_picks, "detector.createPicks");

  NEW_OPT_CLI(config_.url_event_db, "Database", "event-db",
              "load events from the given database or file, format: "
              "[service://]location");

  NEW_OPT_CLI(config_.offline_mode, "Messaging", "offline",
              "offline mode, do not connect to the messaging system (implies "
              "--no-publish i.e. no objects are sent)");
  NEW_OPT_CLI(config_.no_publish, "Messaging", "no-publish",
              "do not send any objects");
  NEW_OPT_CLI(
      config_.path_ep, "Messaging", "ep",
      "same as --no-publish, but outputs all event parameters scml "
      "formatted; specifying the output path as '-' (a single dash) will "
      "force the output to be redirected to stdout");

  NEW_OPT_CLI(config_.playback_config.start_time_str, "Records",
              "record-starttime",
              "defines a start time (YYYY-MM-DDTHH:MM:SS formatted) for "
              "requesting records from the configured archive recordstream; "
              "implicitly enables reprocessing/playback mode");
  NEW_OPT_CLI(config_.playback_config.end_time_str, "Records", "record-endtime",
              "defines an end time (YYYY-MM-DDTHH:MM:SS formatted) for "
              "requesting records from the configured archive recordstream; "
              "implicitly enables reprocessing/playback mode");

  NEW_OPT_CLI(config_.dump_debug_info, "Mode", "debug-info",
              "dump additional debug information (e.g. waveforms, stats etc.)");
  NEW_OPT_CLI(config_.playback_config.enabled, "Mode", "playback",
              "Use playback mode that does not restrict the maximum allowed "
              "data latency");
  NEW_OPT_CLI(config_.load_templates_only, "Mode", "templates-load-only",
              "load templates and exit");
  NEW_OPT_CLI(config_.templates_no_cache, "Mode", "templates-reload",
              "force reloading templates and omit caching waveforms");

  NEW_OPT_CLI(config_.path_template_json, "Input", "templates-json",
              "path to a template configuration file (json-formatted)");
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

  // TODO(damb): Stream sets not taken into consideration. Detectors
  // aren't able to cope with this concept, anyway, yet.
  try {
    std::ifstream ifs{config_.path_template_json};
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ifs, pt);

    for (const auto &template_setting_pt : pt) {
      try {
        TemplateConfig tc{template_setting_pt.second, config_.detector_config,
                          config_.stream_config};

        SCDETECT_LOG_DEBUG("Creating detector processor (id=%s) ... ",
                           tc.detector_id().c_str());

        if (!IsUniqueProcessorId(tc.detector_id()) && config_.dump_debug_info) {
          throw ConfigError{"Non unique detector processor identifier: " +
                            tc.detector_id()};
        }

        auto detector_builder{
            std::move(Detector::Create(tc.detector_id(), tc.origin_id())
                          .set_config(tc.detector_config(),
                                      config_.playback_config.enabled)
                          .set_eventparameters())};

        boost::filesystem::path path_debug_info;
        if (config_.dump_debug_info) {
          Environment *env{Environment::Instance()};
          boost::filesystem::path sc_install_dir{env->installDir()};
          path_debug_info =
              sc_install_dir / settings::kPathTemp / tc.detector_id();

          detector_builder.set_debug_info_dir(path_debug_info);
        }

        std::vector<std::string> stream_ids;
        for (const auto &stream_set : tc) {
          for (const auto &stream_config_pair : stream_set) {

            IsUniqueProcessorId(stream_config_pair.second.template_id);
            try {
              detector_builder.set_stream(stream_config_pair.first,
                                          stream_config_pair.second,
                                          waveform_handler, path_debug_info);
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
        }

        std::shared_ptr<detect::Detector> detector_ptr{
            detector_builder.Build()};
        detector_ptr->set_result_callback(
            [this](const Processor *proc, const Record *rec,
                   const Processor::ResultCPtr &res) {
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

} // namespace detect
} // namespace Seiscomp
