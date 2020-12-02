#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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

  Detector();

public:
  DEFINE_SMARTPOINTER(Detection);
  struct Detection : public Result {

    using SensorLocations = std::vector<DataModel::SensorLocationCPtr>;
    using TemplateResultMetaData =
        std::unordered_map<std::string, Template::MatchResult::MetaData>;

    Detection(const double fit, const Core::Time &time, const double magnitude,
              const double lat, const double lon, const double depth,
              const SensorLocations &sensor_locations,
              const size_t num_stations_associated,
              const size_t num_stations_used,
              const size_t num_channels_associated,
              const size_t num_channels_used,
              const TemplateResultMetaData &template_metadata);

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

    TemplateResultMetaData template_metadata;
  };

  friend class DetectorBuilder;
  static DetectorBuilder Create(const std::string &origin_id);

  void set_filter(Filter *filter) override;

  bool Feed(const Record *rec) override;
  void Reset() override;

protected:
  void Process(StreamState &stream_state, RecordCPtr record,
               const DoubleArray &filtered_data) override;

  void Fill(StreamState &stream_state, RecordCPtr record, size_t n,
            double *samples) override;

  bool EnoughDataReceived(const StreamState &stream_state) const override;

  void StoreTemplateResult(ProcessorCPtr processor, RecordCPtr record,
                           ResultCPtr result);

  void ResetProcessing();

private:
  struct StreamConfig;
  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;

  struct StreamConfig {
    Processor::StreamState stream_state;
    // TODO(damb): Is a stream_config required at this level?
    // - The detector does no filtering -> no gain is applied.
    // -
    // Processing::Stream stream_config;
    std::unique_ptr<RecordSequence> stream_buffer;
    // Template matching processor
    std::unique_ptr<Processor> processor;

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

    using ProcessorStates = std::unordered_map<std::string, ProcessorState>;
    ProcessorStates processor_states;

    struct Result {
      Core::Time origin_time;
      double fit;
      double magnitude;
    } result;

    Core::Time trigger_end;
  };

  StreamConfigs stream_configs_;
  ProcessingState processing_state_;

  DetectorConfig config_;

  DataModel::OriginPtr origin_;
  DataModel::EventPtr event_;
  DataModel::MagnitudePtr magnitude_;

  Core::TimeWindow processed_{};
};

class DetectorBuilder : public Builder<DetectorBuilder> {

public:
  DetectorBuilder(const std::string &origin_id);

  DetectorBuilder &set_config(const DetectorConfig &config);

  DetectorBuilder &set_eventparameters();
  // Set stream related template configuration
  DetectorBuilder &set_stream(const std::string &stream_id,
                              const StreamConfig &stream_config,
                              WaveformHandlerIfacePtr wf_handler);
  DetectorBuilder &
  // Set a callback function for publishing a detection
  set_publish_callback(const Processor::PublishResultCallback &callback);

  operator DetectorPtr();

  friend void swap(DetectorBuilder &lhs, DetectorBuilder &rhs);

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
