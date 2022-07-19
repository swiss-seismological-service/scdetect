#ifndef SCDETECT_APPS_CC_MAGNITUDE_MRELATIVE_H_
#define SCDETECT_APPS_CC_MAGNITUDE_MRELATIVE_H_

#include <seiscomp/datamodel/stationmagnitude.h>

#include "../magnitude_processor.h"
#include "seiscomp/datamodel/databasereader.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class MRelative : public MagnitudeProcessor {
 public:
  struct TemplateMagnitude {
    double value;
  };

  explicit MRelative(TemplateMagnitude templateMagnitude);

  void finalize(DataModel::StationMagnitude* magnitude) const override;

  void setTemplateMagnitude(TemplateMagnitude templateMagnitude);
  const TemplateMagnitude& templateMagnitude() const;

 protected:
  double computeMagnitude(const DataModel::Amplitude* amplitude) override;

 private:
  // The reference template magnitude
  TemplateMagnitude _templateMagnitude;
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_MAGNITUDE_MRELATIVE_H_
