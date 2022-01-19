#ifndef SCDETECT_APPS_CC_DETECTOR_LINKER_STRATEGY_IPP_
#define SCDETECT_APPS_CC_DETECTOR_LINKER_STRATEGY_IPP_

#include <type_traits>
#include <utility>

#include "../../util/factory.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

template <typename... Args>
std::unique_ptr<MergingStrategy> MergingStrategy::Create(
    MergingStrategy::Type mergingStrategyTypeId, Args &&...args) {
  switch (mergingStrategyTypeId) {
    case MergingStrategy::Type::kGreaterEqualAssociationThres:
      return util::create<LinkIfGreaterEqualAssociationThres>(
          std::forward<Args>(args)...);
    case MergingStrategy::Type::kGreaterEqualMergingThres:
      return util::create<LinkIfGreaterEqualMergingThres>(
          std::forward<Args>(args)...);
    case MergingStrategy::Type::kAll:
      return util::create<LinkAllResults>(std::forward<Args>(args)...);
    default:
      return nullptr;
  }
}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_DETECTOR_LINKER_STRATEGY_IPP_
