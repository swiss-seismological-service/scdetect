#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_EVENT_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_EVENT_H_

#include <seiscomp/core/datetime.h>

#include <string>

#include "association.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

struct Event {
  // The time after the event is considered as expired
  Core::Time expired;
  // The final association
  Association association;
  // Time of the reference arrival pick
  Core::Time refPickTime;

  Event(const Core::Time &expired);
  // Feeds the template result `res` to the event in order to be merged
  void feed(const std::string &procId, const Association::TemplateResult &res,
            const POT &pot);
  // Returns the total number of arrivals
  size_t getArrivalCount() const;
  // Returns `true` if the event must be considered as expired
  bool isExpired(const Core::Time &now) const;
};

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_EVENT_H_
