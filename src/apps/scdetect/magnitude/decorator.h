#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_H_

#include "../magnitudeprocessor.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class Decorator : public MagnitudeProcessor {
 public:
  Decorator(MagnitudeProcessor* processor, const std::string& id = "");

  double compute(DataModel::Amplitude* amplitude) override;

 protected:
  MagnitudeProcessor* _processor;
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_H_
