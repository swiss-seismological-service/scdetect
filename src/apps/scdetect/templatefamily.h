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
  struct Member {
    std::string sensorLocationId;

    DataModel::AmplitudeCPtr amplitude;
    DataModel::StationMagnitudeCPtr magnitude;

    boost::optional<double> lowerLimit;
    boost::optional<double> upperLimit;
  };

 public:
  // Defines available magnitude types
  enum class MagnitudeType { kMw, kML };
  // Builds a template family
  //
  // - allows only a single phase to be associated per origin sensor location
  // combination
  class Builder : public detect::Builder<TemplateFamily> {
   public:
    Builder(const TemplateFamilyConfig& templateFamilyConfig);
    // Sets the template family's identifier
    //
    // - if no `id` is passed the identifier from the template family
    // configuration is used, instead
    Builder& setId(const boost::optional<std::string>& id = boost::none);
    // Sets the template family members' limits
    Builder& setLimits();
    // Sets the template family members' magnitudes
    Builder& setStationMagnitudes();
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
    using MapKey = std::pair<OriginId, SensorLocationId>;
    using Members = std::map<MapKey, TemplateFamily::Member>;
    Members _members;

    using MagnitudeTypeMap =
        std::unordered_map<std::string, TemplateFamily::MagnitudeType>;
    static const MagnitudeTypeMap _magnitudeTypeMap;
  };

  friend Builder;
  static Builder Create(const TemplateFamilyConfig& templateFamilyConfig);

  const std::string& id() const;

  const MagnitudeType& magnitudeType() const;

 protected:
  TemplateFamily();

 private:
  using Members = std::vector<Member>;

  Members _members;
  // The template family identifier
  std::string _id;
  // The template family's magnitude type
  MagnitudeType _magnitudeType{MagnitudeType::kMw};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_
