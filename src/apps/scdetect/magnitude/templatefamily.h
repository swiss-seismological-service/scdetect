#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_TEMPLATEFAMILY_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_TEMPLATEFAMILY_H_

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/stationmagnitude.h>

#include <vector>

#include "../templatefamily.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

// Interface for template family based magnitude processors
class TemplateFamilyBased {
 public:
  // Maps an amplitude with a magnitude
  struct AmplitudeMagnitude {
    DataModel::AmplitudeCPtr amplitude;
    DataModel::StationMagnitudeCPtr magnitude;
  };

 private:
  using AmplitudeMagnitudes = std::vector<AmplitudeMagnitude>;

 public:
  virtual ~TemplateFamilyBased() = default;
  // Adds a single amplitude magnitude pair which is going to be used for the
  // *amplitude-magnitude regression*
  //
  // - neither amplitudes nor magnitudes are validated to be consistent i.e. it
  // is up to the client to add consistent amplitudes and magnitudes)
  virtual void addAmplitudeMagnitude(DataModel::AmplitudeCPtr amplitude,
                                     DataModel::StationMagnitudeCPtr magnitude);

  virtual void resetAmplitudeMagnitudes();

  virtual TemplateFamily::MagnitudeType templateFamilyMagnitudeType() = 0;

  using const_iterator = AmplitudeMagnitudes::const_iterator;
  const_iterator begin() const { return _amplitudeMagnitudes.begin(); }
  const_iterator end() const { return _amplitudeMagnitudes.end(); }
  const_iterator cbegin() const { return _amplitudeMagnitudes.cbegin(); }
  const_iterator cend() const { return _amplitudeMagnitudes.cend(); }

 private:
  AmplitudeMagnitudes _amplitudeMagnitudes;
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_TEMPLATEFAMILY_H_
