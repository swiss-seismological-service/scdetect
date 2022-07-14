#ifndef SCDETECT_APPS_CC_AMPLITUDE_FACTORY_H_
#define SCDETECT_APPS_CC_AMPLITUDE_FACTORY_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "../binding.h"
#include "../detector/detector.h"
#include "../factory.h"
#include "../processing/stream_config.h"

namespace Seiscomp {
namespace detect {

class AmplitudeProcessor;

namespace amplitude {

class RMSAmplitude;
class MLx;

namespace factory {

struct SensorLocationDetectionInfo {
  using WaveformStreamId = std::string;
  struct Pick {
    WaveformStreamId authorativeWaveformStreamId;
    DataModel::PickCPtr pick;
  };

  using ProcessorId = std::string;
  using PickMap = std::unordered_map<ProcessorId, Pick>;
  PickMap pickMap;

  // Sensor location identifier which must include both band and source code
  // identifiers
  std::string sensorLocationStreamId;

  DataModel::OriginCPtr origin;
};

struct AmplitudeProcessorConfig {
  std::string id;

  Core::TimeSpan gapThreshold;
  Core::TimeSpan gapTolerance;
  bool gapInterpolation;
};

struct SensorLocationTimeInfo {
  struct TimeInfo {
    Core::TimeSpan leading;
    Core::TimeSpan trailing;
  };

  using WaveformStreamId = std::string;
  using TimeInfos = std::map<WaveformStreamId, TimeInfo>;
  TimeInfos timeInfos;
};

std::unique_ptr<amplitude::MLx> createMLx(
    const binding::Bindings& bindings,
    const SensorLocationDetectionInfo& sensorLocationDetectionInfo,
    const SensorLocationTimeInfo& SensorLocationTimeInfo,
    const AmplitudeProcessorConfig& amplitudeProcessorConfig);

namespace detail {

const binding::SensorLocationConfig& loadSensorLocationConfig(
    const binding::Bindings& bindings, const std::string& netCode,
    const std::string& staCode, const std::string& locCode,
    const std::string& chaCode);

std::unique_ptr<RMSAmplitude> createRMSAmplitude(
    const binding::Bindings& bindings,
    const SensorLocationDetectionInfo& sensorLocationDetectionInfo,
    const Core::TimeWindow& timeWindow,
    const AmplitudeProcessorConfig& amplitudeProcessorConfig,
    const processing::StreamConfig& streamConfig,
    const std::string& baseProcessorId = "");

}  // namespace detail
}  // namespace factory

class Factory
    : public detect::Factory<AmplitudeProcessor, std::string,
                             const binding::Bindings&,
                             const factory::SensorLocationDetectionInfo&,
                             const detector::Detector&> {
 public:
  class BaseException : public Exception {
   public:
    using Exception::Exception;
    BaseException();
  };

  static std::unique_ptr<detect::AmplitudeProcessor> createMRelative(
      const binding::Bindings& bindings,
      const factory::SensorLocationDetectionInfo& sensorLocationDetectionInfo,
      const detector::Detector& detector);

  static std::unique_ptr<AmplitudeProcessor> createMLx(
      const binding::Bindings& bindings,
      const factory::SensorLocationDetectionInfo& sensorLocationDetectionInfo,
      const detector::Detector& detector);

  // Resets the factory
  static void reset();

 private:
  static std::unique_ptr<AmplitudeProcessor> createRatioAmplitude(
      const binding::Bindings& bindings,
      const factory::SensorLocationDetectionInfo& sensorLocationDetectionInfo,
      const detector::Detector& detector,
      const std::string& baseProcessorId = "");
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_FACTORY_H_
