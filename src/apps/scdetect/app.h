#ifndef SCDETECT_APPS_SCDETECT_APP_H_
#define SCDETECT_APPS_SCDETECT_APP_H_

#include <seiscomp/client/application.h>
#include <seiscomp/client/streamapplication.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/processing/streambuffer.h>
#include <seiscomp/system/commandline.h>

#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "amplitudeprocessor.h"
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

    std::string amplitudeMessagingGroup{"AMPLITUDE"};

    // default configurations
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

  bool isEventDatabaseEnabled() const;

  void emitDetection(
      const detector::DetectorWaveformProcessor *processor,
      const Record *record,
      const detector::DetectorWaveformProcessor::DetectionCPtr &detection);

  void emitAmplitude(const AmplitudeProcessor *processor, const Record *record,
                     const AmplitudeProcessor::AmplitudeCPtr &amplitude);

 protected:
  // Load events either from `eventDb` or `db`
  virtual bool loadEvents(const std::string &eventDb,
                          DataModel::DatabaseQueryPtr db);

 private:
  using Picks = std::vector<DataModel::PickCPtr>;
  // Initialize detectors
  //
  // - `ifs` references a template configuration input file stream
  bool initDetectors(std::ifstream &ifs, WaveformHandlerIface *waveformHandler);

  // Initialize amplitude processors
  bool initAmplitudeProcessors(
      const detector::DetectorWaveformProcessor *processor,
      const detector::DetectorWaveformProcessor::DetectionCPtr &detection,
      const DataModel::OriginCPtr &origin, const Picks &picks);

  // Registers an amplitude processor
  void registerAmplitudeProcessor(
      const std::shared_ptr<ReducingAmplitudeProcessor> &processor);
  // Removes an amplitude processor
  void removeAmplitudeProcessor(
      const std::shared_ptr<ReducingAmplitudeProcessor> &processor);

  Config _config;

  ObjectLog *_outputOrigins;
  ObjectLog *_outputAmplitudes;

  DataModel::EventParametersPtr _ep;

  using WaveformStreamId = std::string;
  using DetectorMap = std::unordered_multimap<
      WaveformStreamId, std::shared_ptr<detector::DetectorWaveformProcessor>>;
  DetectorMap _detectors;

  // Ringbuffer
  Processing::StreamBuffer _waveformBuffer{30 * 60 /*seconds*/};

  using AmplitudeProcessors =
      std::unordered_multimap<WaveformStreamId,
                              std::shared_ptr<ReducingAmplitudeProcessor>>;
  AmplitudeProcessors _amplitudeProcessors;

  struct AmplitudeProcessorQueueItem {
    std::shared_ptr<ReducingAmplitudeProcessor> amplitudeProcessor;
  };
  using AmplitudeProcessorQueue = std::list<AmplitudeProcessorQueueItem>;
  // The queue used for amplitude processor registration
  AmplitudeProcessorQueue _amplitudeProcessorQueue;
  // The queue used for amplitude processor removal
  AmplitudeProcessorQueue _amplitudeProcessorRemovalQueue;
  bool _registrationBlocked{false};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_APP_H_
