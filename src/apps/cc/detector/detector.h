#ifndef SCDETECT_APPS_CC_DETECTOR_DETECTOR_H_
#define SCDETECT_APPS_CC_DETECTOR_DETECTOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/defs.h>
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
#include "../processing/waveform_processor.h"
#include "../waveform.h"
#include "detector_impl.h"
#include "linker/strategy.h"
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

   protected:
    void finalize() override;

   private:
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

    std::string _originId;

    using TemplateProcessorConfigs =
        std::unordered_map<std::string, TemplateProcessorConfig>;
    TemplateProcessorConfigs _processorConfigs;

    static const std::unordered_map<std::string, linker::MergingStrategy::Type>
        _mergingStrategyLookupTable;
  };

  friend class Builder;
  static Builder Create(const std::string &originId);

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

 protected:
  WaveformProcessor::StreamState *streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool store(const Record *record) override;

  void reset(StreamState &streamState) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  // Callback function storing `res`
  void storeDetection(const DetectorImpl::Result &res);
  // Creates a detection from `res`
  std::unique_ptr<const Detection> createDetection(
      const DetectorImpl::Result &res);

  void emitDetection(const Record *record,
                     std::unique_ptr<const Detection> detection);

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
