#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_MRELATIVE_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_MRELATIVE_H_

#include <seiscomp/datamodel/stationmagnitude.h>

#include "../magnitude_processor.h"
#include "seiscomp/datamodel/databasereader.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class MRelative : public MagnitudeProcessor {
 public:
  explicit MRelative(
      DataModel::StationMagnitudeCPtr templateMagnitude = nullptr);

  void finalize(DataModel::StationMagnitude* magnitude) const override;

  void setTemplateMagnitude(DataModel::StationMagnitudeCPtr templateMagnitude);

 protected:
  double computeMagnitude(const DataModel::Amplitude* amplitude) override;

 private:
  // The reference template magnitude
  DataModel::StationMagnitudeCPtr _templateMagnitude;
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_MRELATIVE_H_
