#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/defs.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/origin.h>

#include <boost/optional/optional.hpp>
#include <string>
#include <unordered_map>

#include "../config/detector.h"
#include "../processing/waveform_processor.h"
#include "detector.h"
#include "detectorbuilder.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Detector waveform processor implementation
class DetectorWaveformProcessor : public processing::WaveformProcessor {
  explicit DetectorWaveformProcessor(const DataModel::OriginCPtr &origin);

 public:
  DEFINE_SMARTPOINTER(Detection);
  struct Detection : public Core::BaseObject {
    double fit{};

    Core::Time time;
    double latitude{};
    double longitude{};
    double depth{};

    size_t numStationsAssociated{};
    size_t numStationsUsed{};
    size_t numChannelsAssociated{};
    size_t numChannelsUsed{};

    config::PublishConfig publishConfig;

    using TemplateResult = Detector::Result::TemplateResult;
    using WaveformStreamId = std::string;
    using TemplateResults =
        std::unordered_multimap<WaveformStreamId, TemplateResult>;
    // Template specific results
    TemplateResults templateResults;
  };
  using PublishDetectionCallback = std::function<void(
      const DetectorWaveformProcessor *, const Record *, DetectionCPtr)>;

  friend class DetectorBuilder;
  static DetectorBuilder Create(const std::string &originId);

  // Sets the `callback` in order to publish detections
  void setResultCallback(const PublishDetectionCallback &callback);

  void reset() override;
  void terminate() override;

  const config::PublishConfig &publishConfig() const;

 protected:
  WaveformProcessor::StreamState &streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  void reset(StreamState &streamState) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  // Callback function storing `res`
  void storeDetection(const Detector::Result &res);
  // Prepares the detection from `res`
  void prepareDetection(DetectionPtr &d, const Detector::Result &res);

  void emitDetection(const Record *record, const DetectionCPtr &detection);

 private:
  using WaveformStreamID = std::string;
  using StreamStates =
      std::unordered_map<WaveformStreamID,
                         processing::WaveformProcessor::StreamState>;
  StreamStates _streamStates;

  config::DetectorConfig _config;

  Detector _detector;

  PublishDetectionCallback _detectionCallback;

  boost::optional<Detector::Result> _detection;

  // Reference to the *template* origin
  DataModel::OriginCPtr _origin;
  // Reference to the *template* event
  DataModel::EventPtr _event;

  config::PublishConfig _publishConfig;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_
