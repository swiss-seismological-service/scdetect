#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_H_

#include <memory>

#include "../magnitudeprocessor.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

class Decorator : public MagnitudeProcessor {
 public:
  Decorator(std::unique_ptr<MagnitudeProcessor>&& decorated);

  double compute(const DataModel::Amplitude* amplitude) override;

 protected:
  std::unique_ptr<MagnitudeProcessor> _decorated;
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_H_
