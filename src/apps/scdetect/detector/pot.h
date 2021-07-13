#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_POT_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_POT_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/pick.h>

#include <boost/optional.hpp>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>

#include "arrival.h"

namespace Seiscomp {
namespace detect {
namespace detector {

struct PickOffsetNode {
  PickOffsetNode(const std::string &waveformStreamId, double pickOffset = 0);
  PickOffsetNode(const std::string &waveformStreamId, const Core::Time &lhs,
                 const Core::Time &rhs);

  // The waveform stream identifier
  std::string waveformStreamId;
  // Defines the relative pick offset
  double pickOffset{0};

  bool enabled{true};

  void enable();
  void disable();
};

/* ------------------------------------------------------------------------- */
// The Pick Offset Table (POT)
class PickOffsetTable {
 public:
  struct ArrivalPick {
    DataModel::ArrivalCPtr arrival;
    DataModel::PickCPtr pick;
  };

  PickOffsetTable();
  PickOffsetTable(const std::vector<Arrival> &arrivals);
  PickOffsetTable(const std::vector<ArrivalPick> &arrivals);
  virtual ~PickOffsetTable();

  using Offsets = std::vector<PickOffsetNode>;
  using TableType = std::vector<Offsets>;

  using size_type = TableType::size_type;
  using value_type = TableType::value_type;
  using reference = value_type &;
  using const_reference = const value_type &;
  using iterator = TableType::iterator;
  using const_iterator = TableType::const_iterator;

  // Returns the number of picks stored
  size_type size() const noexcept;
  // Returns `true` if the POT is empty, else `false`
  bool empty() const noexcept;
  reference at(size_type n);
  const_reference at(size_type n) const;

  iterator begin();
  iterator end();
  const_iterator begin() const;
  const_iterator end() const;
  const_iterator cbegin() const;
  const_iterator cend() const;

  // Returns the maximum relative pick offset if defined
  boost::optional<double> pickOffset() const;
  // Returns sorted pick offset nodes with regards to the node identified by
  // `waveformStreamId`; throws if out of bounds
  const_reference getOffsets(const std::string &waveformStreamId) const;
  // Returns sorted pick offset nodes with regards to the node at position `n`;
  // throws if out of bounds
  const_reference getOffsets(size_type n) const;
  // Returns the set waveform stream identifiers of the POT's nodes
  std::unordered_set<std::string> getWaveformStreamIds() const;

  // Enable all entries within the table
  void enable();
  // Enables the pick identified by `waveformStreamId` in the table
  void enable(const std::string &waveformStreamId);
  // Enables all picks with waveform stream ids from the set
  void enable(const std::unordered_set<std::string> &waveformStreamIds);
  // Disable all entries within the table
  void disable();
  // Disables the pick identified by `waveformStreamId` in the table
  void disable(const std::string &waveformStreamId);
  // Disables all picks with waveform stream ids from the set
  void disable(const std::unordered_set<std::string> &waveformStreamIds);

  template <typename TFunc>
  void traverse(const TFunc &func);

 private:
  static std::vector<Arrival> convert(const std::vector<ArrivalPick> &picks);

  TableType _offsetTable;

  bool _enabled{true};
};

using POT = PickOffsetTable;
/* ------------------------------------------------------------------------- */
// Validate pick offsets of two POTs where pick offsets must be smaller than
// `thres`. Pick offsets exceeding the the threshold are returned in
// `exceeded`.
// Returns if the validation was successful (`true`) or not (`false`),
// respectively.
bool validatePickOffsets(const POT &lhs, const POT &rhs,
                         std::unordered_set<std::string> &exceeded,
                         double thres = 0);

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_POT_H_
