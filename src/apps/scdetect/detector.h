#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/sensorlocation.h>

#include <boost/optional.hpp>

#include "builder.h"
#include "config.h"
#include "detector/arrival.h"
#include "detector/detector.h"
#include "detector/pot.h"
#include "detector/template.h"
#include "settings.h"
#include "waveform.h"
#include "waveformoperator.h"
#include "waveformprocessor.h"

namespace Seiscomp {
namespace detect {

class DetectorBuilder;

// Detector waveform processor implementation
class Detector : public WaveformProcessor {

  Detector(const std::string &id, const DataModel::OriginCPtr &origin);

public:
  DEFINE_SMARTPOINTER(Detection);
  struct Detection : public Result {
    double fit{};

    Core::Time time{};
    double latitude{};
    double longitude{};
    double depth{};

    double magnitude{};

    size_t num_stations_associated{};
    size_t num_stations_used{};
    size_t num_channels_associated{};
    size_t num_channels_used{};

    // Indicates if arrivals should be appended to the detection
    bool with_arrivals{false};

    using TemplateResult = detector::Detector::Result::TemplateResult;
    using TemplateResults =
        std::unordered_multimap<std::string, TemplateResult>;
    // Template specific results
    TemplateResults template_results;

    // List of theoretical template arrivals
    std::vector<detector::Arrival> theoretical_template_arrivals;
  };

  friend class DetectorBuilder;
  static DetectorBuilder Create(const std::string &detector_id,
                                const std::string &origin_id);

  void set_filter(Filter *filter, const Core::TimeSpan &init_time) override;

  const Core::TimeWindow &processed() const override;

  void Reset() override;
  void Terminate() override;

protected:
  WaveformProcessor::StreamState &stream_state(const Record *record) override;

  void Process(StreamState &stream_state, const Record *record,
               const DoubleArray &filtered_data) override;

  void Reset(StreamState &stream_state, const Record *record) override;

  void Fill(StreamState &stream_state, const Record *record,
            DoubleArrayPtr &data) override;

  bool EnoughDataReceived(const StreamState &stream_state) const override;
  // Callback function storing `res`
  void StoreDetection(const detector::Detector::Result &res);
  // Prepares the detection from `res`
  void PrepareDetection(DetectionPtr &d, const detector::Detector::Result &res);

private:
  using WaveformStreamID = std::string;
  using StreamStates =
      std::unordered_map<WaveformStreamID, WaveformProcessor::StreamState>;
  StreamStates stream_states_;

  DetectorConfig config_;

  detector::Detector detector_;
  boost::optional<detector::Detector::Result> detection_;

  DataModel::OriginCPtr origin_;
  DataModel::EventPtr event_;
  DataModel::MagnitudePtr magnitude_;

  // List of reference theoretical template arrivals
  std::vector<detector::Arrival> ref_theoretical_template_arrivals_;
};

class DetectorBuilder : public Builder<Detector> {
public:
  DetectorBuilder(const std::string &id, const std::string &origin_id);

  DetectorBuilder &set_config(const DetectorConfig &config, bool playback);

  DetectorBuilder &set_eventparameters();
  // Set stream related template configuration where `stream_id` refers to the
  // waveform stream identifier of the stream to be processed.
  DetectorBuilder &
  set_stream(const std::string &stream_id, const StreamConfig &stream_config,
             WaveformHandlerIfacePtr &wf_handler,
             const boost::filesystem::path &path_debug_info = "");
  // Set the path to the debug info directory
  DetectorBuilder &set_debug_info_dir(const boost::filesystem::path &path);

protected:
  void Finalize() override;

  bool IsValidArrival(const DataModel::ArrivalCPtr arrival,
                      const DataModel::PickCPtr pick);

private:
  struct TemplateProcessorConfig {
    // Template matching processor
    std::unique_ptr<detector::Template> processor;

    struct MetaData {
      // The template's sensor location associated
      DataModel::SensorLocationCPtr sensor_location;
      // The template related pick
      DataModel::PickCPtr pick;
      // The template related arrival
      DataModel::ArrivalCPtr arrival;
    } metadata;
  };

  std::string origin_id_;

  std::vector<detector::POT::ArrivalPick> arrival_picks_;

  using TemplateProcessorConfigs =
      std::unordered_map<std::string, TemplateProcessorConfig>;
  TemplateProcessorConfigs processor_configs_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_DETECTOR_H_
