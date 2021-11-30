#include "horizontal_components.h"

#include "../exception.h"
#include "waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace util {

HorizontalComponents::HorizontalComponents(Client::Inventory *inventory,
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

  if (realSize() != 2) {
    reset();
    throw Exception{
        "failed to load components from inventory: missing components"};
  }

  const auto &streamCode{_threeComponents.comps[0]->code()};
  _channelCode = streamCode.substr(0, 2);
}

HorizontalComponents::NEStreamIterator HorizontalComponents::begin() const {
  return NEStreamIterator{this};
}

HorizontalComponents::NEStreamIterator HorizontalComponents::end() const {
  NEStreamIterator ret{this};
  ret._pos = 3;
  return ret;
}

const std::string &HorizontalComponents::netCode() const {
  return _networkCode;
}

const std::string &HorizontalComponents::staCode() const {
  return _stationCode;
}

const std::string &HorizontalComponents::locCode() const {
  return _locationCode;
}

const std::string &HorizontalComponents::chaCode() const {
  return _channelCode;
}

bool operator==(const HorizontalComponents &lhs,
                const HorizontalComponents &rhs) {
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

bool operator!=(const HorizontalComponents &lhs,
                const HorizontalComponents &rhs) {
  return !(lhs == rhs);
}

size_t HorizontalComponents::realSize() const {
  size_t retval{2};

  for (int i = 1; i < 3; ++i) {
    if (!_threeComponents.comps[i]) {
      --retval;
    }
  }
  return retval;
}

void HorizontalComponents::reset() {
  _networkCode.clear();
  _stationCode.clear();
  _locationCode.clear();
  _channelCode.clear();

  for (size_t i = 0; i < 3; ++i) {
    _threeComponents.comps[i] = nullptr;
  }
}

std::string getSensorLocationStreamId(const HorizontalComponents &c) {
  return c.netCode() + WaveformStreamID::delimiter + c.staCode() +
         WaveformStreamID::delimiter + c.locCode();
}

std::string getWaveformStreamId(const HorizontalComponents &c) {
  return c.netCode() + WaveformStreamID::delimiter + c.staCode() +
         WaveformStreamID::delimiter + c.locCode() +
         WaveformStreamID::delimiter + c.chaCode();
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
