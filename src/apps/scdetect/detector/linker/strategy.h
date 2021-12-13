#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_STRATEGY_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_STRATEGY_H_

#include <memory>

#include "association.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

// Abstract interface for linker merging strategies
//
// - linker merging strategies define which `TemplateWaveformProcessor` results
// are going to be used for linking
class MergingStrategy {
 public:
  enum class Type {
    kGreaterEqualAssociationThres,
    kGreaterEqualMergingThres,
    kAll
  };

  template <typename... Args>
  static std::unique_ptr<MergingStrategy> Create(Type mergingStrategyTypeId,
                                                 Args &&...args);

  virtual bool operator()(const Association::TemplateResult &result,
                          double associationThres, double mergingThres) = 0;
};

// Strategy which will try to merge all results fed to the linker
class LinkAllResults : public MergingStrategy {
 public:
  bool operator()(const Association::TemplateResult &result,
                  double associationThres, double mergingThres) override;
};

// Strategy which will only link the result if >= the association threshold
class LinkIfGreaterEqualAssociationThres : public MergingStrategy {
 public:
  bool operator()(const Association::TemplateResult &result,
                  double associationThres, double mergingThres) override;
};

// Strategy which will only link the result if >= the merging threshold
class LinkIfGreaterEqualMergingThres : public MergingStrategy {
 public:
  bool operator()(const Association::TemplateResult &result,
                  double associationThres, double mergingThres) override;
};

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#include "strategy.ipp"

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_STRATEGY_H_
