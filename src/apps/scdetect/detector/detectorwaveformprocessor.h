#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/defs.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/origin.h>

#include <boost/optional/optional.hpp>
#include <string>
#include <unordered_map>

#include "../config.h"
#include "../waveformoperator.h"
#include "../waveformprocessor.h"
#include "detector.h"
#include "detectorbuilder.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Detector waveform processor implementation
class DetectorWaveformProcessor : public WaveformProcessor {
  DetectorWaveformProcessor(const DataModel::OriginCPtr &origin);

 public:
  DEFINE_SMARTPOINTER(Detection);
  struct Detection : public Result {
    double fit{};

    Core::Time time{};
    double latitude{};
    double longitude{};
    double depth{};

    size_t numStationsAssociated{};
    size_t numStationsUsed{};
    size_t numChannelsAssociated{};
    size_t numChannelsUsed{};

    PublishConfig publishConfig;

    using TemplateResult = Detector::Result::TemplateResult;
    // Maps the waveform stream identifier with the template result
    using TemplateResults =
        std::unordered_multimap<std::string, TemplateResult>;
    // Template specific results
    TemplateResults templateResults;
  };

  friend class DetectorBuilder;
  static DetectorBuilder Create(const std::string &originId);

  void setFilter(Filter *filter, const Core::TimeSpan &initTime) override;

  void reset() override;
  void terminate() override;

  const PublishConfig &publishConfig() const;

 protected:
  WaveformProcessor::StreamState &streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  void reset(StreamState &streamState) override;

  bool fill(detect::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  // Callback function storing `res`
  void storeDetection(const Detector::Result &res);
  // Prepares the detection from `res`
  void prepareDetection(DetectionPtr &d, const Detector::Result &res);

 private:
  using WaveformStreamID = std::string;
  using StreamStates =
      std::unordered_map<WaveformStreamID, WaveformProcessor::StreamState>;
  StreamStates _streamStates;

  DetectorConfig _config;

  Detector _detector;
  boost::optional<Detector::Result> _detection;

  // Reference to the *template* origin
  DataModel::OriginCPtr _origin;
  // Reference to the *template* event
  DataModel::EventPtr _event;

  PublishConfig _publishConfig;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_
