#ifndef SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_
#define SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_

#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/stationmagnitude.h>

#include <boost/optional/optional.hpp>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "amplitudeprocessor.h"
#include "binding.h"
#include "builder.h"
#include "config.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

// Implements a template family
class TemplateFamily {
 public:
  // Defines available magnitude types
  enum class MagnitudeType { kMw, kML };

  // Defines a template family member
  struct Member {
    DataModel::AmplitudeCPtr amplitude;
    DataModel::StationMagnitudeCPtr magnitude;

    struct Config {
      std::string sensorLocationId;
      boost::optional<std::string> detectorId;
      boost::optional<double> lowerLimit;
      boost::optional<double> upperLimit;
    } config;

    // Returns whether the member references a detector
    bool referencesDetector() const;
  };

  using Members = std::vector<Member>;
  // Builds a template family
  //
  // - allows only a single phase to be associated per origin sensor location
  // combination
  class Builder : public detect::Builder<TemplateFamily> {
   public:
    Builder(const TemplateFamilyConfig& templateFamilyConfig);
    // Sets the template family's identifier
    //
    // - allows to explicitly ovveride the `id` (by default the identifier from
    // the template configuration is used, instead)
    Builder& setId(const boost::optional<std::string>& id = boost::none);
    // Sets the template family members' limits
    //
    // - allows parameters to be explicitly overriden from the template family
    // configuration (by means of passing `lower` and `upper`)
    Builder& setLimits(const boost::optional<double>& lower = boost::none,
                       const boost::optional<double>& upper = boost::none);
    // Sets the template family members' magnitudes
    //
    // - allows to explicitly override the `magnitudeType` (by default the
    // magnitude type from the template family configuration is used, instead)
    Builder& setStationMagnitudes(
        const boost::optional<std::string>& magnitudeType = boost::none);
    // Sets the template family members' amplitudes
    Builder& setAmplitudes(WaveformHandlerIface* waveformHandler,
                           const binding::Bindings& binding);

   protected:
    void finalize() override;

   private:
    void storeAmplitude(const AmplitudeProcessor* processor,
                        const Record* record,
                        const AmplitudeProcessor::AmplitudeCPtr& amplitude);

    TemplateFamilyConfig _templateFamilyConfig;

    using SensorLocationId = std::string;
    using OriginId = std::string;
    using Members =
        std::map<OriginId, std::map<SensorLocationId, TemplateFamily::Member>>;
    Members _members;

    using MagnitudeTypeMap =
        std::unordered_map<std::string, TemplateFamily::MagnitudeType>;
    static const MagnitudeTypeMap _magnitudeTypeMap;
  };

  friend Builder;
  static Builder Create(const TemplateFamilyConfig& templateFamilyConfig);

  const std::string& id() const;

  const MagnitudeType& magnitudeType() const;

  Members::const_iterator begin() const noexcept { return _members.cbegin(); }
  Members::const_iterator end() const noexcept { return _members.cend(); }

 protected:
  TemplateFamily();

 private:
  Members _members;
  // The template family identifier
  std::string _id;
  // The template family's magnitude type
  MagnitudeType _magnitudeType{MagnitudeType::kMw};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_
