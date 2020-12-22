#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/sensorlocation.h>

#include "builder.h"
#include "config.h"
#include "processor.h"
#include "settings.h"
#include "template.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

class DetectorBuilder;

DEFINE_SMARTPOINTER(Detector);
class Detector : public Processor {

  Detector(const std::string &id);

public:
  DEFINE_SMARTPOINTER(Detection);
  struct Detection : public Result {

    using SensorLocations = std::vector<DataModel::SensorLocationCPtr>;
    struct TemplateResult {
      // Template specific lag required for setting the pick time
      Core::Time lag;
      // Template related metadata
      Template::MatchResult::MetaData metadata;
    };

    using TemplateResults = std::unordered_map<std::string, TemplateResult>;

    Detection(const double fit, const Core::Time &time, const double magnitude,
              const double lat, const double lon, const double depth,
              const SensorLocations &sensor_locations,
              const size_t num_stations_associated,
              const size_t num_stations_used,
              const size_t num_channels_associated,
              const size_t num_channels_used,
              const TemplateResults &template_results);

    double fit{};

    Core::Time time{};
    double magnitude{};
    double latitude{};
    double longitude{};
    double depth{};

    SensorLocations sensor_locations;

    size_t num_stations_associated{};
    size_t num_stations_used{};
    size_t num_channels_associated{};
    size_t num_channels_used{};

    // Template specific results
    TemplateResults template_results;
  };

  friend class DetectorBuilder;
  static DetectorBuilder Create(const std::string &detector_id,
                                const std::string &origin_id);

  void set_filter(Filter *filter) override;

  bool Feed(const Record *rec) override;
  void Reset() override;

  std::string DebugString() const override;

  bool WithPicks() const override;

protected:
  void Process(StreamState &stream_state, RecordCPtr record,
               const DoubleArray &filtered_data) override;

  void Fill(StreamState &stream_state, RecordCPtr record, size_t n,
            double *samples) override;

  bool EnoughDataReceived(const StreamState &stream_state) const override;

  void StoreTemplateResult(ProcessorCPtr processor, RecordCPtr record,
                           ResultCPtr result);

  void ResetProcessing();

  // Reset template (child) processors
  void ResetProcessors();

private:
  using WaveformStreamID = std::string;

  struct StreamConfig;
  using StreamConfigs = std::unordered_map<WaveformStreamID, StreamConfig>;

  struct StreamConfig {
    Processor::StreamState stream_state;
    // TODO(damb): Is a stream_config required at this level?
    // - The detector does no filtering -> no gain is applied.
    // -
    // Processing::Stream stream_config;
    std::unique_ptr<RecordSequence> stream_buffer;
    // Template matching processor
    ProcessorPtr processor;

    struct MetaData {
      DataModel::SensorLocationCPtr sensor_location;
    } metadata;
  };

  struct ProcessingState {

    struct ProcessorState {

      ProcessorCPtr processor;
      Template::MatchResultCPtr result{nullptr};

      RecordCPtr trace;
    };

    using ProcessorStates =
        std::unordered_map<WaveformStreamID, ProcessorState>;
    ProcessorStates processor_states;

    struct Result {
      using Lags = std::unordered_map<WaveformStreamID, double>;

      Core::Time origin_time;
      double fit{-1};
      double magnitude{0};

      // Lag related data
      Core::TimeWindow time_window;
      Lags lags;

    } result;

    Core::Time trigger_end;
  };

  StreamConfigs stream_configs_;
  ProcessingState processing_state_;

  DetectorConfig config_;

  DataModel::OriginPtr origin_;
  DataModel::EventPtr event_;
  DataModel::MagnitudePtr magnitude_;

  std::multimap<WaveformStreamID, Template::MatchResultCPtr> debug_cc_results_;
};

class DetectorBuilder : public Builder<DetectorBuilder> {

public:
  DetectorBuilder(const std::string &detector_id, const std::string &origin_id);

  DetectorBuilder &set_config(const DetectorConfig &config);

  DetectorBuilder &set_eventparameters();
  // Set stream related template configuration
  DetectorBuilder &
  set_stream(const std::string &stream_id, const StreamConfig &stream_config,
             WaveformHandlerIfacePtr wf_handler,
             const boost::filesystem::path &path_debug_info = "");
  DetectorBuilder &
  // Set a callback function for publishing a detection
  set_publish_callback(const Processor::PublishResultCallback &callback);
  // Set the path to the debug info directory
  DetectorBuilder &set_debug_info_dir(const boost::filesystem::path &path);

  DetectorPtr build();

protected:
  bool IsValidArrival(const DataModel::ArrivalCPtr arrival,
                      const DataModel::PickCPtr pick);

  bool set_origin(const std::string &origin_id);

private:
  std::string origin_id_;

  DetectorPtr detector_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_DETECTOR_H_
