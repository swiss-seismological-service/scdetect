#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/defs.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>

#include <boost/optional.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "../config.h"
#include "../waveformoperator.h"
#include "../waveformprocessor.h"
#include "arrival.h"
#include "detector.h"
#include "detectorbuilder.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Detector waveform processor implementation
class DetectorWaveformProcessor : public WaveformProcessor {
  DetectorWaveformProcessor(const std::string &id,
                            const DataModel::OriginCPtr &origin);

 public:
  DEFINE_SMARTPOINTER(Detection);
  struct Detection : public Result {
    double fit{};

    Core::Time time{};
    double latitude{};
    double longitude{};
    double depth{};

    double magnitude{};

    size_t numStationsAssociated{};
    size_t numStationsUsed{};
    size_t numChannelsAssociated{};
    size_t numChannelsUsed{};

    PublishConfig publishConfig;

    // Indicates if arrivals should be appended to the detection
    bool withArrivals{false};

    using TemplateResult = Detector::Result::TemplateResult;
    using TemplateResults =
        std::unordered_multimap<std::string, TemplateResult>;
    // Template specific results
    TemplateResults templateResults;

    // List of theoretical template arrivals
    std::vector<Arrival> theoreticalTemplateArrivals;
  };

  friend class DetectorBuilder;
  static DetectorBuilder Create(const std::string &detectorId,
                                const std::string &originId);

  void setFilter(Filter *filter, const Core::TimeSpan &initTime) override;

  const Core::TimeWindow &processed() const override;

  void reset() override;
  void terminate() override;

 protected:
  WaveformProcessor::StreamState &streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  void reset(StreamState &streamState, const Record *record) override;

  void fill(StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  bool enoughDataReceived(const StreamState &streamState) const override;
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

  DataModel::OriginCPtr _origin;
  DataModel::EventPtr _event;
  DataModel::MagnitudePtr _magnitude;

  PublishConfig _publishConfig;

  // List of reference theoretical template arrivals
  std::vector<Arrival> _refTheoreticalTemplateArrivals;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORWAVEFORMPROCESSOR_H_
