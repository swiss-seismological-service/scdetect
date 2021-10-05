#ifndef SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_
#define SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_

#include <seiscomp/core/record.h>
#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/stationmagnitude.h>

#include <map>
#include <utility>

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
    DataModel::AmplitudeCPtr amplitude;
    DataModel::StationMagnitudeCPtr magnitude;
  };

 public:
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
    // Sets the template family members' magnitudes based on the
    // amplitude/magnitude type `magnitudeType`
    Builder& setStationMagnitudes(const std::string& magnitudeType);
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
  };

  friend Builder;
  static Builder Create(const TemplateFamilyConfig& templateFamilyConfig);

  const std::string& id() const;

 protected:
  TemplateFamily();

 private:
  using Members = std::vector<Member>;

  Members _members;
  // The template family identifier
  std::string _id;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_
