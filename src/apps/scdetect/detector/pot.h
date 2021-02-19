#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_POT_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_POT_H_

#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/optional.hpp>

#include <seiscomp/core/datetime.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/pick.h>

#include "arrival.h"

namespace Seiscomp {
namespace detect {
namespace detector {

struct PickOffsetNode {
  PickOffsetNode(const std::string &stream_id, double pick_offset = 0);
  PickOffsetNode(const std::string &stream_id, const Core::Time &lhs,
                 const Core::Time &rhs);

  // The waveform stream identifier
  std::string stream_id;
  // Defines the relative pick offset
  double pick_offset{0};

  bool enabled{true};

  void Enable();
  void Disable();
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
  boost::optional<double> pick_offset() const;
  // Returns sorted pick offset nodes with regards to the node identified by
  // `stream_id`; throws if out of bounds
  const_reference GetOffsets(const std::string &stream_id) const;
  // Returns sorted pick offset nodes with regards to the node at position `n`;
  // throws if out of bounds
  const_reference GetOffsets(size_type n) const;
  // Returns the set waveform stream identifiers of the POT's nodes
  std::unordered_set<std::string> GetWaveformIDs() const;

  // Enable all entries within the table
  void Enable();
  // Enables the pick identified by `stream_id` in the table
  void Enable(const std::string &stream_id);
  // Enables all picks with waveform stream ids from the set
  void Enable(const std::unordered_set<std::string> &stream_ids);
  // Disable all entries within the table
  void Disable();
  // Disables the pick identified by `stream_id` in the table
  void Disable(const std::string &stream_id);
  // Disables all picks with waveform stream ids from the set
  void Disable(const std::unordered_set<std::string> &stream_ids);

  template <typename TFunc> void Traverse(const TFunc &func);

private:
  static std::vector<Arrival> Convert(const std::vector<ArrivalPick> &picks);

  TableType offset_table_;

  bool enabled_{true};
};

using POT = PickOffsetTable;
/* ------------------------------------------------------------------------- */
// Validate pick offsets of two POTs where pick offsets must be smaller than
// `thres`. Pick offsets exceeding the the threshold are returned in
// `exceeded`.
// Returns if the validation was successful (`true`) or not (`false`),
// respectively.
bool ValidatePickOffsets(const POT &lhs, const POT &rhs,
                         std::unordered_set<std::string> &exceeded,
                         double thres = 0);

} // namespace detector
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_DETECTOR_POT_H_
