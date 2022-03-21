#ifndef SCDETECT_APPS_CC_DETECTOR_DETECTOR_H_
#define SCDETECT_APPS_CC_DETECTOR_DETECTOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/sensorlocation.h>

#include <boost/optional/optional.hpp>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>

#include "../builder.h"
#include "../config/detector.h"
#include "../def.h"
#include "../processing/waveform_processor.h"
#include "../waveform.h"
#include "detector_impl.h"
#include "seiscomp/core/typedarray.h"
#include "template_waveform_processor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Detector waveform processor implementation
class Detector : public processing::WaveformProcessor {
  explicit Detector(const DataModel::OriginCPtr &origin);

 public:
  struct Detection {
    double score{};

    Core::Time time;
    double latitude{};
    double longitude{};
    double depth{};

    size_t numStationsAssociated{};
    size_t numStationsUsed{};
    size_t numChannelsAssociated{};
    size_t numChannelsUsed{};

    config::PublishConfig publishConfig;

    using TemplateResult = DetectorImpl::Result::TemplateResult;
    using TemplateResults = DetectorImpl::Result::TemplateResults;
    // Template specific results
    TemplateResults templateResults;
  };

  using PublishDetectionCallback = std::function<void(
      const Detector *, const Record *, std::unique_ptr<const Detection>)>;

  class Builder : public detect::Builder<Detector> {
   public:
    explicit Builder(const std::string &originId);

    Builder &setId(const std::string &id);

    Builder &setConfig(const config::PublishConfig &publishConfig,
                       const config::DetectorConfig &detectorConfig,
                       bool playback);

    // Set stream related template configuration where `streamId` refers to the
    // waveform stream identifier of the stream to be processed.
    Builder &setStream(const std::string &streamId,
                       const config::StreamConfig &streamConfig,
                       WaveformHandlerIface *waveformHandler);

    Builder &setExecutor(std::shared_ptr<Executor> executor);

   protected:
    void finalize() override;

   private:
    void setMergingStrategy(const std::string &strategyId);

    static bool isValidArrival(const DataModel::Arrival &arrival,
                               const DataModel::Pick &pick);

    struct TemplateProcessorConfig {
      // Template matching processor
      std::unique_ptr<TemplateWaveformProcessor> processor;
      // `TemplateWaveformProcessor` specific merging threshold
      boost::optional<double> mergingThreshold;

      struct MetaData {
        // The template's sensor location associated
        DataModel::SensorLocationCPtr sensorLocation;
        // The template related pick
        DataModel::PickCPtr pick;
        // The template related arrival
        DataModel::ArrivalCPtr arrival;
      } metadata;
    };

    std::shared_ptr<Executor> _executor;

    std::string _originId;

    using TemplateProcessorConfigs =
        std::unordered_map<std::string, TemplateProcessorConfig>;
    TemplateProcessorConfigs _processorConfigs;
  };

  friend class Builder;
  static Builder Create(const std::string &originId);

  void setGapInterpolation(bool gapInterpolation) override;
  void setGapThreshold(const Core::TimeSpan &duration) override;
  void setGapTolerance(const Core::TimeSpan &duration) override;

  // Sets the `callback` in order to publish detections
  void setResultCallback(const PublishDetectionCallback &callback);

  void reset() override;
  void terminate() override;

  const config::PublishConfig &publishConfig() const;

  // Returns the underlying template waveform processor identified by
  // `processorId`
  //
  // - returns a `nullptr` if no processor is identified by `processorId`
  const TemplateWaveformProcessor *processor(
      const std::string &processorId) const;

  using const_iterator = DetectorImpl::const_iterator;
  const_iterator begin() const { return _detectorImpl.begin(); }
  const_iterator end() const { return _detectorImpl.end(); }
  const_iterator cbegin() const { return _detectorImpl.cbegin(); }
  const_iterator cend() const { return _detectorImpl.cend(); }

 protected:
  WaveformProcessor::StreamState *streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool store(const Record *record) override;

  void reset(StreamState &streamState) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  bool handleGap(processing::StreamState &streamState, const Record *record,
                 DoubleArrayPtr &data) override;

  // Callback function storing `res`
  void storeDetection(const DetectorImpl::Result &res);
  // Creates a detection from `res`
  std::unique_ptr<const Detection> createDetection(
      const DetectorImpl::Result &res);

  void emitDetection(const Record *record,
                     std::unique_ptr<const Detection> detection);

  const DetectorImpl &detectorImpl() const;

 private:
  void processDetections(const Record *record);

  using WaveformStreamID = std::string;
  using StreamStates =
      std::unordered_map<WaveformStreamID,
                         processing::WaveformProcessor::StreamState>;
  StreamStates _streamStates;

  config::DetectorConfig _config;

  DetectorImpl _detectorImpl;

  PublishDetectionCallback _detectionCallback;

  using DetectionQueue = std::list<DetectorImpl::Result>;
  DetectionQueue _detectionQueue;

  // Reference to the *template* origin
  DataModel::OriginCPtr _origin;

  config::PublishConfig _publishConfig;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_DETECTOR_DETECTOR_H_
