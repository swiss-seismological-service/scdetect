#include "three_components.h"

#include "../exception.h"
#include "../settings.h"

namespace Seiscomp {
namespace detect {
namespace util {

ThreeComponents::ThreeComponents(Client::Inventory *inventory,
                                 const std::string &netCode,
                                 const std::string &staCode,
                                 const std::string &locCode,
                                 const std::string &chaCode,
                                 const Core::Time &time)
    : _networkCode{netCode}, _stationCode{staCode}, _locationCode{locCode} {
  try {
    _threeComponents =
        inventory->getThreeComponents(netCode, staCode, locCode, chaCode, time);
  } catch (Core::ValueException &e) {
    reset();
    throw Exception{"failed to load components from inventory: " +
                    std::string{e.what()}};
  }

  if (realSize() != 3) {
    reset();
    throw Exception{
        "failed to load components from inventory: missing components"};
  }

  const auto &streamCode{_threeComponents.comps[0]->code()};
  _channelCode = streamCode.substr(0, 2);
}

ThreeComponents::ZNEStreamIterator ThreeComponents::begin() const {
  return ZNEStreamIterator{this};
}

ThreeComponents::ZNEStreamIterator ThreeComponents::end() const {
  ZNEStreamIterator ret{this};
  ret._pos = 3;
  return ret;
}

const std::string &ThreeComponents::netCode() const { return _networkCode; }

const std::string &ThreeComponents::staCode() const { return _stationCode; }

const std::string &ThreeComponents::locCode() const { return _locationCode; }

const std::string &ThreeComponents::chaCode() const { return _channelCode; }

std::string ThreeComponents::sensorLocationStreamId() const {
  return _networkCode + settings::kSNCLSep + _stationCode + settings::kSNCLSep +
         _locationCode;
}

std::string ThreeComponents::waveformStreamId() const {
  return _networkCode + settings::kSNCLSep + _stationCode + settings::kSNCLSep +
         _locationCode + settings::kSNCLSep + _channelCode;
}

bool operator==(const ThreeComponents &lhs, const ThreeComponents &rhs) {
  if (lhs._networkCode != rhs._networkCode) {
    return false;
  }
  if (lhs._stationCode != rhs._stationCode) {
    return false;
  }
  if (lhs._locationCode != rhs._locationCode) {
    return false;
  }

  auto lhsIt{lhs.begin()};
  auto rhsIt{rhs.begin()};
  while (lhsIt != lhs.end() || rhsIt != rhs.end()) {
    if ((*lhsIt && !*rhsIt) || (!*lhsIt && *rhsIt) ||
        (*lhsIt && *rhsIt && *lhsIt != *rhsIt)) {
      return false;
    }
    ++lhsIt;
    ++rhsIt;
  }
  return true;
}

bool operator!=(const ThreeComponents &lhs, const ThreeComponents &rhs) {
  return !(lhs == rhs);
}

size_t ThreeComponents::realSize() const {
  size_t retval{3};
  for (int i = 0; i < 3; ++i) {
    if (!_threeComponents.comps[i]) {
      --retval;
    }
  }
  return retval;
}

void ThreeComponents::reset() {
  _networkCode.clear();
  _stationCode.clear();
  _locationCode.clear();
  _channelCode.clear();

  for (size_t i = 0; i < 3; ++i) {
    _threeComponents.comps[i] = nullptr;
  }
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
