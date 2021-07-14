#include "event.h"

#include <algorithm>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

Event::Event(const Core::Time &expired) : expired{expired} {}

void Event::feed(const std::string &procId,
                 const Association::TemplateResult &res, const POT &pot) {
  auto &templateResults{association.results};
  templateResults.emplace(procId, res);

  std::vector<double> fits;
  std::transform(std::begin(templateResults), std::end(templateResults),
                 std::back_inserter(fits),
                 [](const Association::TemplateResults::value_type &p) {
                   return p.second.matchResult->coefficient;
                 });

  // compute the overall event's score
  association.fit = utils::cma(fits.data(), fits.size());
  association.pot = pot;
  if (!refPickTime || res.arrival.pick.time < refPickTime) {
    refPickTime = res.arrival.pick.time;
    association.refProcId = procId;
  }
}

size_t Event::getArrivalCount() const { return association.results.size(); }

bool Event::isExpired(const Core::Time &now) const { return now >= expired; }

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
