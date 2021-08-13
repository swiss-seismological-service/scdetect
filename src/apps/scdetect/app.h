#ifndef SCDETECT_APPS_SCDETECT_APP_H_
#define SCDETECT_APPS_SCDETECT_APP_H_

#include <seiscomp/client/application.h>
#include <seiscomp/client/streamapplication.h>
#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/system/commandline.h>

#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"
#include "detector/detectorwaveformprocessor.h"
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

  class DuplicatePublicObjectId : public BaseException {
   public:
    using BaseException::BaseException;
    DuplicatePublicObjectId();
  };

  struct Config {
    Config();

    void init(const Client::Application *app);
    void init(const System::CommandLine &commandline);

    std::string pathFilesystemCache;
    std::string urlEventDb;

    bool templatesPrepare{false};
    bool templatesNoCache{false};

    // Defines if a detector should be initialized although template
    // processors could not be initialized due to missing waveform data.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipTemplateIfNoWaveformData{true};
    // Defines if a detector should be initialized although template processors
    // could not be initialized due to missing stream information in the
    // inventory.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipTemplateIfNoStreamData{true};
    // Defines if a detector should be initialized although template processors
    // could not be initialized due to missing sensor location information in
    // the inventory.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipTemplateIfNoSensorLocationData{true};

    // Input
    std::string pathTemplateJson{};

    // Reprocessing / playback
    struct {
      std::string startTimeStr;
      std::string endTimeStr;

      Core::Time startTime;
      Core::Time endTime;

      // Indicates if playback mode is enabled/disabled
      bool enabled{false};
    } playbackConfig;

    // Messaging
    bool offlineMode{false};
    bool noPublish{false};
    std::string pathEp;

    PublishConfig publishConfig;

    DetectorConfig detectorConfig;

    StreamConfig streamConfig;
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

  void emitDetection(
      const detector::DetectorWaveformProcessor *processor,
      const Record *record,
      const detector::DetectorWaveformProcessor::DetectionCPtr &detection);

 protected:
  // Load events either from `eventDb` or `db`
  virtual bool loadEvents(const std::string &eventDb,
                          DataModel::DatabaseQueryPtr db);

 private:
  // Initialize detectors
  //
  // - `ifs` references a template configuration input file stream
  bool initDetectors(std::ifstream &ifs, WaveformHandlerIface *waveformHandler);

  Config _config;
  ObjectLog *_outputOrigins;

  DataModel::EventParametersPtr _ep;

  using DetectorMap = std::unordered_multimap<
      std::string, std::shared_ptr<detector::DetectorWaveformProcessor>>;
  DetectorMap _detectors;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_APP_H_
