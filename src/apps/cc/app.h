#ifndef SCDETECT_APPS_CC_APP_H_
#define SCDETECT_APPS_CC_APP_H_

#include <seiscomp/client/application.h>
#include <seiscomp/client/monitor.h>
#include <seiscomp/client/streamapplication.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/stationmagnitude.h>
#include <seiscomp/processing/streambuffer.h>
#include <seiscomp/system/commandline.h>

#include <boost/optional/optional.hpp>
#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "amplitude_processor.h"
#include "binding.h"
#include "config/detector.h"
#include "config/template_family.h"
#include "detector/detector.h"
#include "exception.h"
#include "processing/timewindow_processor.h"
#include "settings.h"
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

    // Flag with forces the waveform buffer size
    boost::optional<Core::TimeSpan> forcedWaveformBufferSize{
        Core::TimeSpan{300.0}};

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

    // Monitoring
    boost::optional<std::size_t> objectThroughputInfoThreshold;
    boost::optional<std::size_t> objectThroughputWarningThreshold;

    boost::optional<std::size_t> objectThroughputNofificationInterval;

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

  bool dispatch(Core::BaseObject *obj) override;

  void handleTimeout() override;

  void handleRecord(Record *rec) override;

  using Detectors = std::vector<std::unique_ptr<detector::Detector>>;
  const Detectors &detectors() const;
  // Reset detectors
  void resetDetectors();

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

    using ProcessorId = std::string;
    using Amplitudes = std::unordered_map<ProcessorId, DataModel::AmplitudePtr>;
    Amplitudes amplitudes;
    using Magnitudes = std::vector<DataModel::StationMagnitudePtr>;
    Magnitudes magnitudes;
    using NetworkMagnitudes = std::vector<DataModel::MagnitudePtr>;
    NetworkMagnitudes networkMagnitudes;

    struct ArrivalPick {
      DataModel::ArrivalPtr arrival;
      DataModel::PickPtr pick;
    };
    using ArrivalPicks = std::vector<ArrivalPick>;
    // Picks and arrivals which are associated to the detection (i.e. both
    // detected picks and *template picks*)
    ArrivalPicks arrivalPicks;

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
    std::shared_ptr<const detector::Detector::Detection> detection;

    std::size_t numberOfRequiredAmplitudes{};
    std::size_t numberOfRequiredMagnitudes{};

    bool published{false};

    const std::string &id() const { return origin->publicID(); }

    bool amplitudesReady() const {
      std::size_t count{};
      for (const auto &amplitudePair : amplitudes) {
        if (amplitudePair.second) {
          ++count;
        }
      }
      return numberOfRequiredAmplitudes == count;
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
  // Initialize network magnitudes
  static bool initNetworkMagnitudes(const TemplateConfigs &templateConfigs);
  // Initialize template families
  //
  // - `ifs` references a template family configuration input file stream
  static bool initTemplateFamilies(std::ifstream &ifs,
                                   WaveformHandlerIface *waveformHandler,
                                   const TemplateConfigs &templateConfigs,
                                   const binding::Bindings &bindings,
                                   const Config &appConfig);

  static Core::TimeSpan computeWaveformBufferSize(
      const TemplateConfigs &templateConfigs, const binding::Bindings &bindings,
      const Config &appConfig);

  using NetworkMagnitudeComputationStrategy =
      std::function<void(const std::vector<DataModel::StationMagnitudeCPtr> &,
                         DataModel::Magnitude &)>;
  static const NetworkMagnitudeComputationStrategy
      medianNetworkMagnitudeComputationStrategy;

  bool isEventDatabaseEnabled() const;

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
  bool initAmplitudeProcessors(std::shared_ptr<DetectionItem> &detectionItem,
                               const detector::Detector &detectorProcessor);

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

  // Computes the network magnitudes based `stationMagnitudes`
  std::vector<DataModel::MagnitudePtr> createNetworkMagnitudes(
      const std::vector<DataModel::StationMagnitudeCPtr> &stationMagnitudes,
      NetworkMagnitudeComputationStrategy strategy,
      const std::string &methodId = "", const std::string &processorId = "");

  using WaveformStreamId = std::string;
  // Registers an amplitude `processor` for `waveformStreamIds`
  void registerAmplitudeProcessor(
      const std::shared_ptr<AmplitudeProcessor> &processor,
      DetectionItem &detection);
  // Registers a time window `processor` for `waveformStreamIds`
  void registerTimeWindowProcessor(
      const std::vector<WaveformStreamId> &waveformStreamIds,
      const std::shared_ptr<processing::TimeWindowProcessor> &);
  // Unregisters time window `processor`
  void removeTimeWindowProcessor(
      const std::shared_ptr<processing::TimeWindowProcessor> &processor);

  // Registers a detection
  void registerDetection(const std::shared_ptr<DetectionItem> &detection);
  // Removes a detection
  void removeDetection(const std::shared_ptr<DetectionItem> &detection);

  void processDetection(
      const detector::Detector *processor, const Record *record,
      std::unique_ptr<const detector::Detector::Detection> detection);

  void publishDetection(const std::shared_ptr<DetectionItem> &detection);
  void publishDetection(const DetectionItem &detectionItem);

  void publishAndRemoveDetection(std::shared_ptr<DetectionItem> &detection);

  std::unique_ptr<DataModel::Comment> createTemplateWaveformTimeInfoComment(
      const detector::Detector::Detection::TemplateResult &templateResult);

  Config _config;
  binding::Bindings _bindings;

  ObjectLog *_outputOrigins;
  ObjectLog *_outputAmplitudes;

  DataModel::EventParametersPtr _ep;

  Detectors _detectors;

  using DetectorIdx = std::unordered_multimap<WaveformStreamId, std::size_t>;
  DetectorIdx _detectorIdx;

  // Ringbuffer
  Processing::StreamBuffer _waveformBuffer;

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

  // Used to monitor the average object throughput
  Client::RunningAverage _averageObjectThroughputMonitor{
      settings::kObjectThroughputAverageTimeSpan};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_APP_H_
