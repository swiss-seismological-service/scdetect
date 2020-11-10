#ifndef SCDETECT_APPS_SCDETECT_APP_H_
#define SCDETECT_APPS_SCDETECT_APP_H_

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <seiscomp/client/streamapplication.h>

#include "config.h"
#include "processor.h"
#include "version.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

class Application : public Client::StreamApplication {

  using StreamDetectorMap = std::unordered_multimap<std::string, ProcessorPtr>;

public:
  Application(int argc, char **argv);
  ~Application(){};

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

    bool load_templates_only{false};

    // Input
    std::string path_template_json{};

    // Messaging
    bool offline_mode{false};
    bool no_publish{false};

    DetectorConfig detector_config;

    StreamConfig stream_config;
  };

  virtual const char *version() override { return detect::kVersion; }

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

  void EmitDetection(ProcessorCPtr processor, RecordCPtr record,
                     Processor::ResultCPtr result);

private:
  void SetupConfigurationOptions();
  bool InitDetectors(WaveformHandlerIfacePtr waveform_handler);

  Options options_;
  Config config_;
  ObjectLog *output_origins_;

  StreamDetectorMap detectors_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_APP_H_
