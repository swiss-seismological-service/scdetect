#ifndef SCDETECT_APPS_SCDETECT_APP_H_
#define SCDETECT_APPS_SCDETECT_APP_H_

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <seiscomp/client/streamapplication.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>

#include "config.h"
#include "exception.h"
#include "waveform.h"
#include "waveformprocessor.h"

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

  DEFINE_SMARTPOINTER(Option);
  struct Option : public Core::BaseObject {
    Option(const char *cfg_name, const char *cli_group = nullptr,
           const char *cli_param = nullptr, const char *cli_desc = nullptr,
           bool cli_default = false, bool cli_switch = false)
        : cfg_name(cfg_name), cli_group(cli_group), cli_param(cli_param),
          cli_desc(cli_desc), cli_default(cli_default), cli_switch(cli_switch) {
    }

    virtual void Bind(System::CommandLine *cli) = 0;
    virtual bool Get(System::CommandLine *cli) = 0;
    virtual bool Get(const Client::Application *app) = 0;
    virtual void PrintStorage(std::ostream &os) = 0;

    const char *cfg_name;
    const char *cli_group;
    const char *cli_param;
    const char *cli_desc;
    bool cli_default;
    bool cli_switch;
  };

  using Options = std::list<OptionPtr>;

  struct Config {

    std::string path_filesystem_cache;
    std::string url_event_db;

    bool load_templates_only{false};
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
      bool enabled;
    } playback_config;

    // Messaging
    bool offline_mode{false};
    bool no_publish{false};
    std::string path_ep{"-"};

    // Debugging
    // Defines if debug information (e.g. waveforms, stats etc.) is dumped
    bool dump_debug_info{false};

    DetectorConfig detector_config;

    StreamConfig stream_config;
  };

  const char *version() override;

protected:
  void createCommandLineDescription() override;
  bool validateParameters() override;
  bool initConfiguration() override;

  void AddOption(OptionPtr);

  void AddOption(int *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(double *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(bool *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(std::string *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(std::vector<int> *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(std::vector<double> *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(std::vector<bool> *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  void AddOption(std::vector<std::string> *var, const char *cfg_name,
                 const char *cli_group = nullptr,
                 const char *cli_param = nullptr,
                 const char *cli_desc = nullptr, bool cli_default = false,
                 bool cli_switch = false);

  const Options &options() const;

  bool init() override;
  bool run() override;
  void done() override;

  void handleRecord(Record *rec) override;

  void EmitDetection(const WaveformProcessor *processor, const Record *record,
                     const WaveformProcessor::ResultCPtr &result);

protected:
  // Load events either from `event_db` or `db`.
  virtual bool LoadEvents(const std::string &event_db,
                          DataModel::DatabaseQueryPtr db);

private:
  void SetupConfigurationOptions();
  bool InitDetectors(WaveformHandlerIfacePtr waveform_handler);

  Options options_;
  Config config_;
  ObjectLog *output_origins_;

  DataModel::EventParametersPtr ep_;

  using DetectorMap =
      std::unordered_multimap<std::string, std::shared_ptr<WaveformProcessor>>;
  DetectorMap detectors_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_APP_H_
