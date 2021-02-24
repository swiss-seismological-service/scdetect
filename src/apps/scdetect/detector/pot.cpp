#include "pot.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace detector {

PickOffsetNode::PickOffsetNode(const std::string &stream_id, double pick_offset)
    : stream_id{stream_id}, pick_offset{pick_offset} {}
PickOffsetNode::PickOffsetNode(const std::string &stream_id,
                               const Core::Time &lhs, const Core::Time &rhs)
    : stream_id{stream_id}, pick_offset{std::abs(rhs - lhs)} {}

void PickOffsetNode::Enable() { enabled = true; }
void PickOffsetNode::Disable() { enabled = false; }

/* ------------------------------------------------------------------------- */
PickOffsetTable::PickOffsetTable() {}

PickOffsetTable::PickOffsetTable(const std::vector<Arrival> &arrivals) {
  // TODO(damb): Currently, a POT cannot handle multiple arrivals per stream.
  // Ensure that waveform ids are unique.
  auto sorted{arrivals};
  std::sort(sorted.begin(), sorted.end(),
            [](const Arrival &lhs, const Arrival &rhs) {
              return lhs.pick.time < rhs.pick.time;
            });

  for (auto i{sorted.cbegin()}; i != sorted.cend(); ++i) {
    Offsets offsets{PickOffsetNode{i->pick.waveform_id}};
    for (auto j{i + 1}; j != sorted.cend(); ++j) {
      PickOffsetNode node{j->pick.waveform_id, j->pick.time, i->pick.time};
      offsets.push_back(node);
    }

    offset_table_.push_back(offsets);
  }
}

PickOffsetTable::PickOffsetTable(
    const std::vector<PickOffsetTable::ArrivalPick> &arrivals)
    : PickOffsetTable{Convert(arrivals)} {}

PickOffsetTable::~PickOffsetTable() {}

PickOffsetTable::size_type PickOffsetTable::size() const noexcept {
  return offset_table_.size();
}

bool PickOffsetTable::empty() const noexcept { return offset_table_.empty(); }

PickOffsetTable::reference PickOffsetTable::at(size_type n) {
  return offset_table_.at(n);
}

PickOffsetTable::const_reference PickOffsetTable::at(size_type n) const {
  return offset_table_.at(n);
}

PickOffsetTable::iterator PickOffsetTable::begin() {
  return offset_table_.begin();
}

PickOffsetTable::iterator PickOffsetTable::end() { return offset_table_.end(); }

PickOffsetTable::const_iterator PickOffsetTable::begin() const {
  return offset_table_.begin();
}

PickOffsetTable::const_iterator PickOffsetTable::end() const {
  return offset_table_.end();
}

PickOffsetTable::const_iterator PickOffsetTable::cbegin() const {
  return offset_table_.cbegin();
}

PickOffsetTable::const_iterator PickOffsetTable::cend() const {
  return offset_table_.cend();
}

boost::optional<double> PickOffsetTable::pick_offset() const {
  if (offset_table_.size()) {
    auto it{offset_table_.cbegin()};

    while (it != --offset_table_.cend()) {
      if (it->at(0).enabled) {
        break;
      }
      ++it;
    }

    if (it != offset_table_.cend() - 1) {
      return (--it->cend())->pick_offset;
    }
  }
  return boost::none;
}

PickOffsetTable::const_reference
PickOffsetTable::GetOffsets(const std::string &stream_id) const {
  const auto it{
      std::find_if(begin(), end(), [&stream_id](const Offsets &offsets) {
        return offsets.at(0).stream_id == stream_id;
      })};

  return at(std::distance(begin(), it));
}

PickOffsetTable::const_reference
PickOffsetTable::GetOffsets(size_type n) const {
  return at(n);
}

std::unordered_set<std::string> PickOffsetTable::GetWaveformIDs() const {
  std::unordered_set<std::string> ret;
  for (const auto &offsets : offset_table_) {
    ret.emplace(offsets.at(0).stream_id);
  }
  return ret;
}

void PickOffsetTable::Enable() {
  if (!enabled_) {
    Traverse([](PickOffsetNode &n) { n.Enable(); });
  }
  enabled_ = true;
}

void PickOffsetTable::Enable(const std::string &stream_id) {
  if (!enabled_) {
    auto Enable = [&stream_id](PickOffsetNode &n) {
      if (stream_id == n.stream_id)
        n.Enable();
    };

    Traverse(Enable);
  }
}

void PickOffsetTable::Enable(
    const std::unordered_set<std::string> &stream_ids) {
  if (stream_ids.empty()) {
    return;
  }

  if (!enabled_) {
    auto Enable = [&stream_ids](PickOffsetNode &n) {
      if (stream_ids.find(n.stream_id) != stream_ids.end())
        n.Enable();
    };
    Traverse(Enable);
  }
}

void PickOffsetTable::Disable() {
  Traverse([](PickOffsetNode &n) { n.Disable(); });
  enabled_ = false;
}

void PickOffsetTable::Disable(const std::string &stream_id) {
  auto Disable = [&stream_id](PickOffsetNode &n) {
    if (stream_id == n.stream_id)
      n.Disable();
  };

  Traverse(Disable);
  enabled_ = false;
}

void PickOffsetTable::Disable(
    const std::unordered_set<std::string> &stream_ids) {
  if (stream_ids.empty()) {
    return;
  }

  auto Disable = [&stream_ids](PickOffsetNode &n) {
    if (stream_ids.find(n.stream_id) != stream_ids.end())
      n.Disable();
  };
  Traverse(Disable);
  enabled_ = false;
}

template <typename TFunc> void PickOffsetTable::Traverse(const TFunc &func) {
  TraverseDepthFirst(*this, func);
}

std::vector<Arrival> PickOffsetTable::Convert(
    const std::vector<PickOffsetTable::ArrivalPick> &arrivals) {
  std::vector<Arrival> ret;
  std::transform(arrivals.cbegin(), arrivals.cend(), back_inserter(ret),
                 [](const PickOffsetTable::ArrivalPick &ap) {
                   return Arrival{
                       Pick{ap.pick->time().value(), ap.pick->waveformID()},
                       ap.arrival->phase().code(), ap.arrival->weight()};
                 });
  return ret;
}

/* ------------------------------------------------------------------------- */
template <typename TFunc>
void TraverseDepthFirst(PickOffsetTable &t, const TFunc &func) {
  for (auto &offsets : t) {
    for (auto n : offsets) {
      func(n);
    }
  }
}

bool ValidatePickOffsets(const POT &lhs, const POT &rhs,
                         std::unordered_set<std::string> &exceeded,
                         double thres) {

  const auto GetStreamsEnabled = [](const POT &pot) {
    std::set<std::string> ret;
    const auto nodes{pot.at(0)};
    for (const auto &n : nodes) {
      if (n.enabled) {
        ret.emplace(n.stream_id);
      }
    }
    return ret;
  };

  const auto lhs_stream_ids{GetStreamsEnabled(lhs)};
  const auto rhs_stream_ids{GetStreamsEnabled(rhs)};
  if (lhs_stream_ids != rhs_stream_ids) {
    return false;
  }

  const auto CreateIdx = [](const POT::Offsets &offsets) {
    std::unordered_map<std::string, double> ret;
    for (const auto &n : offsets) {
      if (n.enabled) {
        ret.emplace(n.stream_id, n.pick_offset);
      }
    }
    return ret;
  };

  constexpr double tolerance{1.0e-6};
  exceeded.clear();
  for (const auto &stream_id : lhs_stream_ids) {
    const auto lhs_idx{CreateIdx(lhs.GetOffsets(stream_id))};
    const auto rhs_idx{CreateIdx(rhs.GetOffsets(stream_id))};

    for (const auto &p : lhs_idx) {
      try {
        if (utils::GreaterThan(std::abs(p.second - rhs_idx.at(p.first)), thres,
                               tolerance)) {
          exceeded.emplace(stream_id);
        }
      } catch (std::out_of_range &e) {
        // picks are in reversed order
        const auto alternative_idx{CreateIdx(rhs.GetOffsets(p.first))};
        const auto it{alternative_idx.find(stream_id)};
        if (it == alternative_idx.end()) {
          return false;
        }

        if (utils::GreaterThan(std::abs(p.second + it->second), thres,
                               tolerance)) {
          exceeded.emplace(stream_id);
        }
      }
    }
  }

  return true;
}

} // namespace detector
} // namespace detect
} // namespace Seiscomp
