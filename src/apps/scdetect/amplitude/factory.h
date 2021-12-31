#ifndef SCDETECT_APPS_SCDETECT_AMPLITUDE_FACTORY_H_
#define SCDETECT_APPS_SCDETECT_AMPLITUDE_FACTORY_H_

#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "../binding.h"
#include "../detector/detectorwaveformprocessor.h"
#include "../factory.h"

namespace Seiscomp {
namespace detect {

class AmplitudeProcessor;

namespace amplitude {
namespace factory {

struct Detection {
  using WaveformStreamId = std::string;
  struct Pick {
    WaveformStreamId authorativeWaveformStreamId;
    DataModel::PickCPtr pick;
  };

  using ProcessorId = std::string;
  using PickMap = std::unordered_map<ProcessorId, Pick>;
  PickMap pickMap;

  std::string sensorLocationStreamId;

  DataModel::OriginCPtr origin;
};

}  // namespace factory

class Factory
    : public detect::Factory<AmplitudeProcessor, std::string,
                             const binding::Bindings&,
                             const factory::Detection&,
                             const detector::DetectorWaveformProcessor&> {
 public:
  class BaseException : public Exception {
   public:
    using Exception::Exception;
    BaseException();
  };

  static std::unique_ptr<detect::AmplitudeProcessor> createMRelative(
      const binding::Bindings& bindings, const factory::Detection& detection,
      const detector::DetectorWaveformProcessor& detector);

  static std::unique_ptr<AmplitudeProcessor> createMLx(
      const binding::Bindings& bindings, const factory::Detection& detection,
      const detector::DetectorWaveformProcessor& detector);

 private:
  static const binding::SensorLocationConfig& loadSensorLocationConfig(
      const binding::Bindings& bindings, const std::string& netCode,
      const std::string& staCode, const std::string& locCode,
      const std::string& chaCode);

  static std::unique_ptr<AmplitudeProcessor> createRatioAmplitude(
      const binding::Bindings& bindings, const factory::Detection& detection,
      const detector::DetectorWaveformProcessor& detector);
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_AMPLITUDE_FACTORY_H_
