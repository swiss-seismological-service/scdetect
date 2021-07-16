#include "strategy.h"

#include "../../utils.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

inline bool LinkAllResults::operator()(
    const Association::TemplateResult &result, double associationThres,
    double mergingThres) {
  return true;
}

bool LinkIfGreaterEqualAssociationThres::operator()(
    const Association::TemplateResult &result, double associationThres,
    double mergingThres) {
  return result.matchResult->coefficient >= associationThres;
}

bool LinkIfGreaterEqualMergingThres::operator()(
    const Association::TemplateResult &result, double associationThres,
    double mergingThres) {
  return result.matchResult->coefficient >= mergingThres;
}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
