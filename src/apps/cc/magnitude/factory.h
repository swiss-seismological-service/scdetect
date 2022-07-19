#ifndef SCDETECT_APPS_CC_MAGNITUDE_FACTORY_H_
#define SCDETECT_APPS_CC_MAGNITUDE_FACTORY_H_

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/stationmagnitude.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../binding.h"
#include "../exception.h"
#include "../factory.h"
#include "template_family.h"

namespace Seiscomp {
namespace detect {

class MagnitudeProcessor;

namespace magnitude {

class Factory
    : public detect::Factory<MagnitudeProcessor, std::string,
                             const DataModel::Amplitude&,
                             const binding::Bindings&, const std::string&> {
 public:
  class BaseException : public Exception {
   public:
    using Exception::Exception;
    BaseException();
  };

  static std::unique_ptr<MagnitudeProcessor> createMLx(
      const DataModel::Amplitude& amplitude, const binding::Bindings& bindings,
      const std::string& processorId);

  static std::unique_ptr<MagnitudeProcessor> createMRelative(
      const DataModel::Amplitude& amplitude, const binding::Bindings& bindings,
      const std::string& processorId);

  // Register a template family
  static void add(std::unique_ptr<TemplateFamily>&& templateFamily);
  // Unregister a template family
  static void removeTemplateFamily(const std::string& detectorId,
                                   const std::string& sensorLocationId);

  // Register station `magnitude`
  static void add(const std::string& detectorId,
                  const std::string& sensorLocationId,
                  DataModel::StationMagnitudeCPtr magnitude);
  // Unregister the station magnitude previously registered under `detectorId`
  // and `sensorLocationId`
  static void removeStationMagnitude(const std::string& detectorId,
                                     const std::string& sensorLocationId);

  // Register the network `magnitude`
  static void add(const std::string& detectorId,
                  DataModel::MagnitudeCPtr magnitude);
  // Unregister the network magnitude previously registered under `detectorId`
  static void removeNetworkMagnitude(const std::string& detectorId);

  // Resets the factory
  static void reset();

 private:
  using DetectorId = std::string;
  using SensorLocationId = std::string;
  static bool configureTemplateFamily(
      magnitude::TemplateFamilyBased* processor, const DetectorId& detectorId,
      const SensorLocationId& sensorLocationId,
      const std::string& magnitudeType,
      std::shared_ptr<TemplateFamily>& templateFamily);
  static bool configureLimits(
      std::unique_ptr<MagnitudeProcessor>& ret, const DetectorId& detectorId,
      const SensorLocationId& sensorLocationId,
      const std::shared_ptr<TemplateFamily>& templateFamily);

  using TemplateFamilies = std::unordered_map<
      DetectorId,
      std::unordered_map<SensorLocationId, std::shared_ptr<TemplateFamily>>>;
  static TemplateFamilies& templateFamilies();

  using StationMagnitudes = std::unordered_map<
      DetectorId,
      std::unordered_map<SensorLocationId, DataModel::StationMagnitudeCPtr>>;
  static StationMagnitudes& stationMagnitudes();

  using NetworkMagnitudes =
      std::unordered_map<DetectorId, DataModel::MagnitudeCPtr>;
  static NetworkMagnitudes& networkMagnitudes();
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_MAGNITUDE_FACTORY_H_
