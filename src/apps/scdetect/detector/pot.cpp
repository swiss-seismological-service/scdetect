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

PickOffsetNode::PickOffsetNode(const std::string &waveformStreamId,
                               double pickOffset)
    : waveformStreamId{waveformStreamId}, pickOffset{pickOffset} {}
PickOffsetNode::PickOffsetNode(const std::string &waveformStreamId,
                               const Core::Time &lhs, const Core::Time &rhs)
    : waveformStreamId{waveformStreamId}, pickOffset{std::abs(rhs - lhs)} {}

void PickOffsetNode::enable() { enabled = true; }
void PickOffsetNode::disable() { enabled = false; }

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
    Offsets offsets{PickOffsetNode{i->pick.waveformStreamId}};
    for (auto j{i + 1}; j != sorted.cend(); ++j) {
      PickOffsetNode node{j->pick.waveformStreamId, j->pick.time, i->pick.time};
      offsets.push_back(node);
    }

    _offsetTable.push_back(offsets);
  }
}

PickOffsetTable::PickOffsetTable(
    const std::vector<PickOffsetTable::ArrivalPick> &arrivals)
    : PickOffsetTable{convert(arrivals)} {}

PickOffsetTable::~PickOffsetTable() {}

PickOffsetTable::size_type PickOffsetTable::size() const noexcept {
  return _offsetTable.size();
}

bool PickOffsetTable::empty() const noexcept { return _offsetTable.empty(); }

PickOffsetTable::reference PickOffsetTable::at(size_type n) {
  return _offsetTable.at(n);
}

PickOffsetTable::const_reference PickOffsetTable::at(size_type n) const {
  return _offsetTable.at(n);
}

PickOffsetTable::iterator PickOffsetTable::begin() {
  return _offsetTable.begin();
}

PickOffsetTable::iterator PickOffsetTable::end() { return _offsetTable.end(); }

PickOffsetTable::const_iterator PickOffsetTable::begin() const {
  return _offsetTable.begin();
}

PickOffsetTable::const_iterator PickOffsetTable::end() const {
  return _offsetTable.end();
}

PickOffsetTable::const_iterator PickOffsetTable::cbegin() const {
  return _offsetTable.cbegin();
}

PickOffsetTable::const_iterator PickOffsetTable::cend() const {
  return _offsetTable.cend();
}

boost::optional<double> PickOffsetTable::pickOffset() const {
  if (_offsetTable.size()) {
    auto it{_offsetTable.cbegin()};

    while (it != --_offsetTable.cend()) {
      if (it->at(0).enabled) {
        break;
      }
      ++it;
    }

    if (it != _offsetTable.cend() - 1) {
      return (--it->cend())->pickOffset;
    }
  }
  return boost::none;
}

PickOffsetTable::const_reference PickOffsetTable::getOffsets(
    const std::string &waveformStreamId) const {
  const auto it{
      std::find_if(begin(), end(), [&waveformStreamId](const Offsets &offsets) {
        return offsets.at(0).waveformStreamId == waveformStreamId;
      })};

  return at(std::distance(begin(), it));
}

PickOffsetTable::const_reference PickOffsetTable::getOffsets(
    size_type n) const {
  return at(n);
}

std::unordered_set<std::string> PickOffsetTable::getWaveformStreamIds() const {
  std::unordered_set<std::string> ret;
  for (const auto &offsets : _offsetTable) {
    ret.emplace(offsets.at(0).waveformStreamId);
  }
  return ret;
}

void PickOffsetTable::enable() {
  if (!_enabled) {
    traverse([](PickOffsetNode &n) { n.enable(); });
  }
  _enabled = true;
}

void PickOffsetTable::enable(const std::string &waveformStreamId) {
  if (!_enabled) {
    auto enable = [&waveformStreamId](PickOffsetNode &n) {
      if (waveformStreamId == n.waveformStreamId) n.enable();
    };

    traverse(enable);
  }
}

void PickOffsetTable::enable(
    const std::unordered_set<std::string> &waveformStreamIds) {
  if (waveformStreamIds.empty()) {
    return;
  }

  if (!_enabled) {
    auto enable = [&waveformStreamIds](PickOffsetNode &n) {
      if (waveformStreamIds.find(n.waveformStreamId) != waveformStreamIds.end())
        n.enable();
    };
    traverse(enable);
  }
}

void PickOffsetTable::disable() {
  traverse([](PickOffsetNode &n) { n.disable(); });
  _enabled = false;
}

void PickOffsetTable::disable(const std::string &waveformStreamId) {
  auto disable = [&waveformStreamId](PickOffsetNode &n) {
    if (waveformStreamId == n.waveformStreamId) n.disable();
  };

  traverse(disable);
  _enabled = false;
}

void PickOffsetTable::disable(
    const std::unordered_set<std::string> &waveformStreamIds) {
  if (waveformStreamIds.empty()) {
    return;
  }

  auto disable = [&waveformStreamIds](PickOffsetNode &n) {
    if (waveformStreamIds.find(n.waveformStreamId) != waveformStreamIds.end())
      n.disable();
  };
  traverse(disable);
  _enabled = false;
}

template <typename TFunc>
void PickOffsetTable::traverse(const TFunc &func) {
  traverseDepthFirst(*this, func);
}

std::vector<Arrival> PickOffsetTable::convert(
    const std::vector<PickOffsetTable::ArrivalPick> &picks) {
  std::vector<Arrival> ret;
  std::transform(picks.cbegin(), picks.cend(), back_inserter(ret),
                 [](const PickOffsetTable::ArrivalPick &ap) {
                   return Arrival{
                       Pick{ap.pick->time().value(), ap.pick->waveformID()},
                       ap.arrival->phase().code(), ap.arrival->weight()};
                 });
  return ret;
}

/* ------------------------------------------------------------------------- */
template <typename TFunc>
void traverseDepthFirst(PickOffsetTable &t, const TFunc &func) {
  for (auto &offsets : t) {
    for (auto n : offsets) {
      func(n);
    }
  }
}

bool validatePickOffsets(const POT &lhs, const POT &rhs,
                         std::unordered_set<std::string> &exceeded,
                         double thres) {
  const auto getStreamsEnabled = [](const POT &pot) {
    std::set<std::string> ret;
    const auto nodes{pot.at(0)};
    for (const auto &n : nodes) {
      if (n.enabled) {
        ret.emplace(n.waveformStreamId);
      }
    }
    return ret;
  };

  const auto lhsStreamIds{getStreamsEnabled(lhs)};
  const auto rhsStreamIds{getStreamsEnabled(rhs)};
  if (lhsStreamIds != rhsStreamIds) {
    return false;
  }

  const auto createIdx = [](const POT::Offsets &offsets) {
    std::unordered_map<std::string, double> ret;
    for (const auto &n : offsets) {
      if (n.enabled) {
        ret.emplace(n.waveformStreamId, n.pickOffset);
      }
    }
    return ret;
  };

  constexpr double tolerance{1.0e-6};
  exceeded.clear();
  for (const auto &streamId : lhsStreamIds) {
    const auto lhsIdx{createIdx(lhs.getOffsets(streamId))};
    const auto rhs_idx{createIdx(rhs.getOffsets(streamId))};

    for (const auto &p : lhsIdx) {
      try {
        if (utils::greaterThan(std::abs(p.second - rhs_idx.at(p.first)), thres,
                               tolerance)) {
          exceeded.emplace(streamId);
        }
      } catch (std::out_of_range &e) {
        // picks are in reversed order
        const auto alternativeIdx{createIdx(rhs.getOffsets(p.first))};
        const auto it{alternativeIdx.find(streamId)};
        if (it == alternativeIdx.end()) {
          return false;
        }

        if (utils::greaterThan(std::abs(p.second + it->second), thres,
                               tolerance)) {
          exceeded.emplace(streamId);
        }
      }
    }
  }

  return true;
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
