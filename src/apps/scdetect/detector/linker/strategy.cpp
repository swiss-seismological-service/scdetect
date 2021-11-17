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
  return result.resultIt->coefficient >= associationThres;
}

bool LinkIfGreaterEqualMergingThres::operator()(
    const Association::TemplateResult &result, double associationThres,
    double mergingThres) {
  return result.resultIt->coefficient >= mergingThres;
}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
