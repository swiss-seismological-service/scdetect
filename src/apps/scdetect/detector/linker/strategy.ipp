#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_STRATEGY_IPP_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_STRATEGY_IPP_

#include <type_traits>
#include <utility>

#include "../../util/memory.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

// possible to construct
template <typename T, typename... Args>
typename std::enable_if<std::is_constructible<T, Args...>::value,
                        std::unique_ptr<T> >::type
create(Args &&...args) {
  return util::make_unique<T>(std::forward<Args>(args)...);
}

// impossible to construct
template <typename T, typename... Args>
typename std::enable_if<!std::is_constructible<T, Args...>::value,
                        std::unique_ptr<T> >::type
create(Args &&...) {
  return nullptr;
}

template <typename... Args>
std::unique_ptr<MergingStrategy> MergingStrategy::Create(
    MergingStrategy::Type mergingStrategyTypeId, Args &&...args) {
  switch (mergingStrategyTypeId) {
    case MergingStrategy::Type::kGreaterEqualAssociationThres:
      return create<LinkIfGreaterEqualAssociationThres>(
          std::forward<Args>(args)...);
    case MergingStrategy::Type::kGreaterEqualMergingThres:
      return create<LinkIfGreaterEqualMergingThres>(
          std::forward<Args>(args)...);
    case MergingStrategy::Type::kAll:
      return create<LinkAllResults>(std::forward<Args>(args)...);
    default:
      return nullptr;
  }
}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_STRATEGY_IPP_
