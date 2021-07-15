#include "strategy.h"

#include "../../utils.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

inline bool LinkAllResults::operator()(
    const Association::TemplateResult &result, double associationThres) {
  return true;
}

bool LinkIfGreaterEqualAssociationThres::operator()(
    const Association::TemplateResult &result, double associationThres) {
  return result.matchResult->coefficient >= associationThres;
}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
