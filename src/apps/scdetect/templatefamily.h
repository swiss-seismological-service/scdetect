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

#include "amplitude_processor.h"
#include "binding.h"
#include "builder.h"
#include "config/template_family.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

// Implements a RMS amplitude based template family
class TemplateFamily {
 public:
  // Defines a template family member
  struct Member {
    DataModel::AmplitudeCPtr amplitude;
    DataModel::StationMagnitudeCPtr magnitude;

    struct Config {
      std::string sensorLocationId;
      boost::optional<std::string> detectorId;
      boost::optional<double> lowerLimit;
      boost::optional<double> upperLimit;
    };
    Config config;

    // Returns whether the member references a detector
    bool referencesDetector() const;
  };

 private:
  using Members = std::vector<Member>;

 public:
  // Builds a RMS amplitude based template family
  //
  // - allows only a single phase to be associated per origin sensor location
  // combination
  class Builder : public detect::Builder<TemplateFamily> {
   public:
    explicit Builder(const config::TemplateFamilyConfig& templateFamilyConfig);
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
    //
    // TODO(damb): Allow to inject an amplitude processor (preconfigured)
    Builder& setAmplitudes(WaveformHandlerIface* waveformHandler,
                           const binding::Bindings& binding);

   protected:
    void finalize() override;

   private:
    using SensorLocationId = std::string;
    using OriginId = std::string;

    void storeAmplitude(const AmplitudeProcessor* processor,
                        const Record* record,
                        const AmplitudeProcessor::AmplitudeCPtr& amplitude);

    config::TemplateFamilyConfig _templateFamilyConfig;

    using SensorLocationMap =
        std::map<SensorLocationId, TemplateFamily::Member>;
    using Members = std::map<OriginId, SensorLocationMap>;
    Members _members;

    using MagnitudeTypeMap =
        std::unordered_map<std::string, std::vector<std::string>>;
    // Maps the configured template family magnitude type to *real* station
    // magnitude types
    static const MagnitudeTypeMap _magnitudeTypeMap;
  };

  friend Builder;
  static Builder Create(
      const config::TemplateFamilyConfig& templateFamilyConfig);

  const std::string& id() const;

  const std::string& magnitudeType() const;

  using const_iterator = Members::const_iterator;
  using size_type = Members::size_type;
  const_iterator begin() const noexcept { return _members.begin(); }
  const_iterator end() const noexcept { return _members.end(); }

  size_type size() const noexcept { return _members.size(); }
  bool empty() const noexcept { return _members.empty(); }

 protected:
  TemplateFamily();

 private:
  Members _members;
  // The template family identifier
  std::string _id;
  // The template family's magnitude type
  std::string _magnitudeType{"MLx"};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEMPLATEFAMILY_H_
