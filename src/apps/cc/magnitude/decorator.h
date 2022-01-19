#ifndef SCDETECT_APPS_CC_MAGNITUDE_DECORATOR_H_
#define SCDETECT_APPS_CC_MAGNITUDE_DECORATOR_H_

#include <memory>

#include "../magnitude_processor.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class Decorator : public MagnitudeProcessor {
 public:
  Decorator(std::unique_ptr<MagnitudeProcessor>&& decorated);

  double compute(const DataModel::Amplitude* amplitude) override;

  void finalize(DataModel::StationMagnitude* magnitude) const override;

 protected:
  MagnitudeProcessor* decorated();

 private:
  std::unique_ptr<MagnitudeProcessor> _decorated;
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_MAGNITUDE_DECORATOR_H_
