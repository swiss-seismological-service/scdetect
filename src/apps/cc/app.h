#ifndef SCDETECT_APPS_SCDETECT_APP_H_
#define SCDETECT_APPS_SCDETECT_APP_H_

#include <seiscomp/client/application.h>
#include <seiscomp/client/streamapplication.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/stationmagnitude.h>
#include <seiscomp/processing/streambuffer.h>
#include <seiscomp/system/commandline.h>

#include <boost/optional/optional.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "binding.h"
#include "config/detector.h"
#include "config/template_family.h"
#include "detector/detectorwaveformprocessor.h"
#include "exception.h"
#include "processing/timewindow_processor.h"
#include "reducing_amplitude_processor.h"
#include "util/waveform_stream_id.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

class Application : public Client::StreamApplication {
 public:
  Application(int argc, char **argv);

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
    // Global flag indicating whether to enable `true` or disable `false`
    // calculating amplitudes (regardless of the configuration provided on
    // detector configuration level granularity).
    boost::optional<bool> amplitudesForceMode;
    // Global flag indicating whether to enable `true` or disable `false`
    // calculating magnitudes (regardless of the configuration provided on
    // detector configuration level granularity.
    boost::optional<bool> magnitudesForceMode;

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
    // Defines if a detector should be initialized although template processors
    // could not be initialized due to a missing pick in the event parameters.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipTemplateIfNoPick{true};
    // Defines if a template family should be initialized despite reference
    // configurations could not be initialized due to missing waveform data.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipReferenceConfigIfNoWaveformData{true};
    // Defines if a template family should be initialized although reference
    // configurations could not be initialized due to missing stream
    // information in the inventory.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipReferenceConfigIfNoStreamData{true};
    // Defines if a template family should be initialized although reference
    // configurations could not be initialized due to missing sensor location
    // information in the inventory.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipReferenceConfigIfNoSensorLocationData{true};
    // Defines if a template family should be initialized although reference
    // configurations could not be initialized due to a missing pick in the
    // event parameters.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipReferenceConfigIfNoPick{true};
    // Defines if a template family should be initialized although reference
    // configurations could not be initialized due to missing bindings
    // configuration.
    // XXX(damb): For the time being, this configuration parameter is not
    // provided to module users.
    bool skipReferenceConfigIfNoBindings{true};

    // Input
    std::string pathTemplateJson;
    std::string pathTemplateFamilyJson;

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
    config::PublishConfig publishConfig;

    config::DetectorConfig detectorConfig;

    config::StreamConfig streamConfig;

    config::TemplateFamilyConfig::ReferenceConfig::SensorLocationConfig
        templateFamilySensorLocationConfig;

    // binding default configurations
    binding::SensorLocationConfig sensorLocationBindings;
  };

  const char *version() override;

 protected:
  void createCommandLineDescription() override;
  bool validateParameters() override;
  bool initConfiguration() override;
  bool handleCommandLineOptions() override;

  bool init() override;
  bool run() override;
  void done() override;

  void handleRecord(Record *rec) override;

  bool isEventDatabaseEnabled() const;

 private:
  using Picks = std::vector<DataModel::PickCPtr>;
  using TemplateConfigs = std::vector<config::TemplateConfig>;

  struct DetectionItem {
    explicit DetectionItem(const DataModel::OriginPtr &origin)
        : origin{origin} {
      assert(origin);
    }

    Core::Time expired{Core::Time::GMT() +
                       Core::TimeSpan{10 * 60.0 /*seconds*/}};

    struct ProcessorConfig {
      bool gapInterpolation;
      Core::TimeSpan gapThreshold;
      Core::TimeSpan gapTolerance;
    };

    ProcessorConfig config;

    using Amplitudes = std::vector<DataModel::AmplitudePtr>;
    Amplitudes amplitudes;
    using Magnitudes = std::vector<DataModel::StationMagnitudePtr>;
    Magnitudes magnitudes;

    struct ArrivalPick {
      DataModel::ArrivalPtr arrival;
      DataModel::PickPtr pick;
    };
    using ArrivalPicks = std::vector<ArrivalPick>;
    // Picks and arrivals which are associated to the detection (i.e. both
    // detected picks and *template picks*)
    ArrivalPicks arrivalPicks;

    using ProcessorId = std::string;
    using WaveformStreamId = std::string;
    struct Pick {
      // The authorative full waveform stream identifier
      WaveformStreamId authorativeWaveformStreamId;
      DataModel::PickCPtr pick;
    };
    using AmplitudePickMap = std::unordered_map<ProcessorId, Pick>;
    // Picks used for amplitude calculation
    AmplitudePickMap amplitudePickMap;

    DataModel::OriginPtr origin;

    std::string detectorId;
    detector::DetectorWaveformProcessor::DetectionCPtr detection;

    std::size_t numberOfRequiredAmplitudes{};
    std::size_t numberOfRequiredMagnitudes{};

    bool published{false};

    const std::string &id() const { return origin->publicID(); }

    bool amplitudesReady() const {
      return numberOfRequiredAmplitudes == amplitudes.size();
    }
    bool magnitudesReady() const {
      return numberOfRequiredMagnitudes == magnitudes.size();
    }
    bool ready() const {
      return (amplitudesReady() && magnitudesReady()) ||
             (Core::Time::GMT() >= expired);
    }

    friend bool operator==(const DetectionItem &lhs, const DetectionItem &rhs) {
      return lhs.id() == rhs.id();
    }
    friend bool operator!=(const DetectionItem &lhs, const DetectionItem &rhs) {
      return !(lhs == rhs);
    }
  };

  // Initialize amplitude processor factory
  static bool initAmplitudeProcessorFactory();

  // Initialize magnitude processor factory callbacks
  static bool initMagnitudeProcessorFactory(
      WaveformHandlerIface *waveformHandler,
      const TemplateConfigs &templateConfigs, const binding::Bindings &bindings,
      const Config &appConfig);

  // Initialize station magnitudes
  static bool initStationMagnitudes(const TemplateConfigs &templateConfigs);
  // Initialize template families
  //
  // - `ifs` references a template family configuration input file stream
  static bool initTemplateFamilies(std::ifstream &ifs,
                                   WaveformHandlerIface *waveformHandler,
                                   const TemplateConfigs &templateConfigs,
                                   const binding::Bindings &bindings,
                                   const Config &appConfig);

  // Load events either from `eventDb` or `db`
  bool loadEvents(const std::string &eventDb, DataModel::DatabaseQueryPtr db);

  // Collect required streams
  std::set<util::WaveformStreamID> collectStreams() const;
  // Register `waveformStreamIds` at the record stream
  bool subscribeToRecordStream(
      std::set<util::WaveformStreamID> waveformStreamIds);

  // Initialize detectors
  //
  // - `ifs` references a template configuration input file stream
  bool initDetectors(std::ifstream &ifs, WaveformHandlerIface *waveformHandler,
                     TemplateConfigs &templateConfigs);

  // Initialize amplitude processors
  bool initAmplitudeProcessors(
      std::shared_ptr<DetectionItem> &detectionItem,
      const detector::DetectorWaveformProcessor &detectorProcessor);

  // Creates an amplitude
  //
  // - if `amplitudeType` is passed it overrides the default value
  DataModel::AmplitudePtr createAmplitude(
      const AmplitudeProcessor *processor, const Record *record,
      const AmplitudeProcessor::AmplitudeCPtr &amplitude,
      const boost::optional<std::string> &methodId,
      const boost::optional<std::string> &amplitudeType = boost::none);

  // Computes a magnitude based on `amplitude`
  DataModel::StationMagnitudePtr createMagnitude(
      const DataModel::Amplitude &amplitude, const std::string &methodId = "",
      const std::string &processorId = "");

  using WaveformStreamId = std::string;
  // Registers an amplitude `processor` for `waveformStreamIds`
  void registerAmplitudeProcessor(
      const std::shared_ptr<AmplitudeProcessor> &processor);
  // Registers a time window `processor` for `waveformStreamIds`
  void registerTimeWindowProcessor(
      const std::vector<WaveformStreamId> &waveformStreamIds,
      const std::shared_ptr<processing::TimeWindowProcessor> &);
  // Removes an amplitude processor
  void removeTimeWindowProcessor(
      const std::shared_ptr<processing::TimeWindowProcessor> &processor);

  // Registers a detection
  void registerDetection(const std::shared_ptr<DetectionItem> &detection);
  // Removes a detection
  void removeDetection(const std::shared_ptr<DetectionItem> &detection);

  void processDetection(
      const detector::DetectorWaveformProcessor *processor,
      const Record *record,
      const detector::DetectorWaveformProcessor::DetectionCPtr &detection);

  void publishDetection(const DetectionItem &detectionItem);

  void publishAndRemoveDetection(std::shared_ptr<DetectionItem> &detection);

  std::unique_ptr<DataModel::Comment> createTemplateWaveformTimeInfoComment(
      const detector::DetectorWaveformProcessor::Detection::TemplateResult
          &templateResult);

  Config _config;
  binding::Bindings _bindings;

  ObjectLog *_outputOrigins;
  ObjectLog *_outputAmplitudes;

  DataModel::EventParametersPtr _ep;

  using DetectorMap = std::unordered_multimap<
      WaveformStreamId, std::shared_ptr<detector::DetectorWaveformProcessor>>;
  DetectorMap _detectors;

  // Ringbuffer
  Processing::StreamBuffer _waveformBuffer{30 * 60 /*seconds*/};

  using Detections =
      std::unordered_multimap<WaveformStreamId, std::shared_ptr<DetectionItem>>;
  Detections _detections;

  using DetectionQueue = std::list<std::shared_ptr<DetectionItem>>;
  // The queue used for detection registration
  DetectionQueue _detectionQueue;
  // The queue used for detection removal
  DetectionQueue _detectionRemovalQueue;

  bool _detectionRegistrationBlocked{false};

  using TimeWindowProcessors =
      std::unordered_multimap<WaveformStreamId,
                              std::shared_ptr<processing::TimeWindowProcessor>>;
  TimeWindowProcessors _timeWindowProcessors;
  using ProcessorId = std::string;
  using TimeWindowProcessorIdx =
      std::unordered_map<ProcessorId, std::vector<WaveformStreamId>>;
  TimeWindowProcessorIdx _timeWindowProcessorIdx;

  struct TimeWindowProcessorQueueItem {
    std::vector<WaveformStreamId> waveformStreamIds;
    std::shared_ptr<processing::TimeWindowProcessor> timeWindowProcessor;
  };
  using TimeWindowProcessorQueue = std::list<TimeWindowProcessorQueueItem>;
  // The queue used for time window processor registration
  TimeWindowProcessorQueue _timeWindowProcessorRegistrationQueue;
  // The queue used for time window processor removal
  TimeWindowProcessorQueue _timeWindowProcessorRemovalQueue;
  bool _timeWindowProcessorRegistrationBlocked{false};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_APP_H_