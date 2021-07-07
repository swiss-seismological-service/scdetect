#ifndef SCDETECT_APPS_SCDETECT_APP_H_
#define SCDETECT_APPS_SCDETECT_APP_H_

#include <seiscomp/client/application.h>
#include <seiscomp/client/streamapplication.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/system/commandline.h>

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"
#include "detector.h"
#include "exception.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

class Application : public Client::StreamApplication {
 public:
  Application(int argc, char **argv);
  ~Application() override;

  class BaseException : public Exception {
   public:
    using Exception::Exception;
    BaseException();
  };

  class ConfigError : public BaseException {
   public:
    using BaseException::BaseException;
    ConfigError();
  };

  struct Config {
    Config();

    void Init(const Client::Application *app);
    void Init(const System::CommandLine &commandline);

    std::string path_filesystem_cache;
    std::string url_event_db;

    bool templates_prepare{false};
    bool templates_no_cache{false};

    // Defines if a detector should be initialized although template
    // processors could not be initialized due to missing waveform data.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skip_template_if_no_waveform_data{true};
    // Defines if a detector should be initialized although template processors
    // could not be initialized due to missing stream information in the
    // inventory.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skip_template_if_no_stream_data{true};
    // Defines if a detector should be initialized although template processors
    // could not be initialized due to missing sensor location information in
    // the inventory.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skip_template_if_no_sensor_location_data{true};

    // Input
    std::string path_template_json{};

    // Reprocessing / playback
    struct {
      std::string start_time_str;
      std::string end_time_str;

      Core::Time start_time;
      Core::Time end_time;

      // Indicates if playback mode is enabled/disabled
      bool enabled{false};
    } playback_config;

    // Messaging
    bool offline_mode{false};
    bool no_publish{false};
    std::string path_ep;

    DetectorConfig detector_config;

    StreamConfig stream_config;
  };

  const char *version() override;

 protected:
  void createCommandLineDescription() override;
  bool validateParameters() override;
  bool initConfiguration() override;

  bool init() override;
  bool run() override;
  void done() override;

  void handleRecord(Record *rec) override;

  void EmitDetection(const WaveformProcessor *processor, const Record *record,
                     const WaveformProcessor::ResultCPtr &result);

 protected:
  // Load events either from `event_db` or `db`
  virtual bool LoadEvents(const std::string &event_db,
                          DataModel::DatabaseQueryPtr db);

 private:
  bool InitDetectors(WaveformHandlerIfacePtr waveform_handler);

  Config config_;
  ObjectLog *output_origins_;

  DataModel::EventParametersPtr ep_;

  using DetectorMap =
      std::unordered_multimap<std::string, std::shared_ptr<Detector>>;
  DetectorMap detectors_;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_APP_H_
