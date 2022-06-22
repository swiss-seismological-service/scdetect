#ifndef SCDETECT_APPS_CC_DETECTOR_LINKER_POT_H_
#define SCDETECT_APPS_CC_DETECTOR_LINKER_POT_H_

#include <seiscomp/core/datetime.h>

#include <boost/optional/optional.hpp>
#include <cstddef>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "../detail.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

// The Pick Offset Table (POT)
class POT {
  template <typename T>
  using Matrix = std::vector<std::vector<T>>;
  using OffsetTable = Matrix<double>;

 public:
  struct Entry {
    // The arrival time
    Core::Time arrivalTime;

    std::string templateWaveformProcessorId;
    bool enabled;
  };

  static const double tableDefault;

  POT() = default;
  POT(const std::vector<Entry>& entries);

  using size_type = OffsetTable::size_type;

  size_type size() const noexcept { return _offsets.size(); }
  bool empty() const noexcept { return _offsets.empty(); }

  // Returns the pick offset between `lhsProcessorId` and `rhsProcessorId` if
  // both `lhsProcessorId` and `rhsProcessorId` are enabled
  boost::optional<double> operator()(const std::string& lhsProcessorId,
                                     const std::string& rhsProcessorId) const;
  // Returns whether the processor identified by `processorId` is enabled
  bool enabled(const std::string& processorId) const;
  // Enables the POT for all processors registered
  void enable();
  // Enables the POT for the processor identified by `processorId`
  void enable(const std::string& processorId);
  // Returns whether the processor identified by `processorId` is disabled
  bool disabled(const std::string& processorId) const;
  // Disables the POT for all processors registered
  void disable();
  // Disables the POT for the processor identified by `processorId`
  void disable(const std::string& processorId);

  // Returns the processor identifiers based on the internal sort order
  std::vector<std::string> processorIds() const;

  // Validate pick offsets of this POT with `other` where pick offsets must be
  // smaller than or equal to `thres`.
  //
  // - only validates those offsets which are *enabled* both in `this` and
  // `other`.
  // - returns `true` if the validation was successful or `false` if not,
  // respectively.
  bool validateEnabledOffsets(const POT& other, const Core::TimeSpan& thres);

  // Validates pick offsets of this POT with the `otherOffsets` regarding the
  // processor identified by `processorId` where pick offsets must be smaller
  // than or equal to `thres`.
  //
  // - both the offsets defined by `otherOffsets` and the masks from
  // `otherMask` must be sorted by the corresponding template waveform
  // processor identifier.
  // - only validates those `otherOffsets` which are *enabled* both in `this`
  // and `otherMask`.
  // - returns `true` if the validation was successful or `false` if not,
  // respectively.
  bool validateEnabledOffsets(const std::string& processorId,
                              const std::vector<double>& otherOffsets,
                              const std::vector<bool>& otherMask,
                              const Core::TimeSpan& thres);

 private:
  static const double tolerance;

  bool validEntry(double e) const;

  void setEnable(const std::string& processorId, bool enable);
  void setEnableAll(bool enable);

  Matrix<bool> createMask(
      const std::vector<detail::ProcessorIdType>& enabledProcessors);

  struct Item {
    size_type idx;
    bool enabled;
  };
  using ProcessorIdxMap = std::map<detail::ProcessorIdType, Item>;
  ProcessorIdxMap _processorIdxMap;

  OffsetTable _offsets;
};

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_DETECTOR_LINKER_POT_H_
