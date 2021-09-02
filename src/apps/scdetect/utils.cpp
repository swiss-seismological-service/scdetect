#include "utils.h"

#include <seiscomp/core/exceptions.h>
#include <seiscomp/core/strings.h>
#include <seiscomp/datamodel/stream.h>

#include <boost/algorithm/string.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <sstream>
#include <vector>

#include "exception.h"
#include "settings.h"

namespace Seiscomp {
namespace detect {
namespace utils {

const std::string createUUID() {
  auto uuid{boost::uuids::random_generator{}()};
  return boost::uuids::to_string(uuid);
}

void replaceEscapedXMLFilterIdChars(std::string &str) {
  boost::replace_all(str, "&gt;", ">");
}

/* ------------------------------------------------------------------------- */
std::string to_string(const WaveformStreamID &waveformStreamId) {
  std::ostringstream oss;
  oss << waveformStreamId;
  return oss.str();
}

const std::string WaveformStreamID::_delimiter{settings::kSNCLSep};

WaveformStreamID::WaveformStreamID(const std::string &netStaLocCha) {
  std::vector<std::string> tokens;
  Core::split(tokens, netStaLocCha, _delimiter.c_str(), false);

  if (4 != tokens.size()) {
    throw ValueException{std::string{"Invalid number of tokens: "} +
                         std::to_string(tokens.size())};
  }
  _netCode = tokens[0];
  _staCode = tokens[1];
  _locCode = tokens[2];
  _chaCode = tokens[3];
}

WaveformStreamID::WaveformStreamID(const DataModel::WaveformStreamID &id)
    : _netCode{id.networkCode()},
      _staCode{id.stationCode()},
      _locCode{id.locationCode()},
      _chaCode{id.channelCode()} {
  if (!isValid()) {
    std::ostringstream oss;
    oss << *this;
    throw ValueException{std::string{"Invalid DataModel::WaveformStreamID: "} +
                         oss.str()};
  }
}

WaveformStreamID::WaveformStreamID(const std::string &netCode,
                                   const std::string &staCode,
                                   const std::string &locCode,
                                   const std::string &chaCode)
    : _netCode{netCode},
      _staCode{staCode},
      _locCode{locCode},
      _chaCode{chaCode} {}

const std::string &WaveformStreamID::netCode() const { return _netCode; }
const std::string &WaveformStreamID::staCode() const { return _staCode; }
const std::string &WaveformStreamID::locCode() const { return _locCode; }
const std::string &WaveformStreamID::chaCode() const { return _chaCode; }

std::string WaveformStreamID::sensorLocationStreamId() const {
  return _netCode + _delimiter + _staCode + _delimiter + _locCode;
}

bool WaveformStreamID::isValid() const {
  return !(_netCode.empty() || _staCode.empty() || _chaCode.empty());
}

std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id) {
  os << id._netCode << id._delimiter << id._staCode << id._delimiter
     << id._locCode << id._delimiter << id._chaCode;
  return os;
}

/* ------------------------------------------------------------------------- */
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
    throw Exception{"failed to load components: " + std::string{e.what()}};
  }

  if (realSize() != 3) {
    reset();
    throw Exception{"failed to load components: missing components"};
  }

  const auto &streamCode{_threeComponents.comps[0]->code()};
  _channelCode = streamCode.substr(0, 2);
}

const std::string &ThreeComponents::netCode() const { return _networkCode; }

const std::string &ThreeComponents::staCode() const { return _stationCode; }

const std::string &ThreeComponents::locCode() const { return _locationCode; }

const std::string &ThreeComponents::chaCode() const { return _channelCode; }

std::string ThreeComponents::sensorLocationStreamId() const {
  return _networkCode + settings::kSNCLSep + _stationCode + settings::kSNCLSep +
         _locationCode;
}

std::vector<std::string> ThreeComponents::streamCodes() const {
  std::vector<std::string> retval;
  for (int i = 0; i < 3; ++i) {
    retval.push_back(_threeComponents.comps[i]->code());
  }
  return retval;
}

const DataModel::ThreeComponents &ThreeComponents::threeComponents() const {
  return _threeComponents;
}

std::vector<utils::WaveformStreamID> ThreeComponents::waveformStreamIds()
    const {
  std::vector<utils::WaveformStreamID> retval;
  for (const auto &streamCode : streamCodes()) {
    retval.push_back(utils::WaveformStreamID{_networkCode, _stationCode,
                                             _locationCode, streamCode});
  }
  return retval;
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
  if ((lhs._threeComponents.vertical() && !rhs._threeComponents.vertical()) ||
      (!lhs._threeComponents.vertical() && rhs._threeComponents.vertical()) ||
      (lhs._threeComponents.vertical() && rhs._threeComponents.vertical() &&
       lhs._threeComponents.vertical() != rhs._threeComponents.vertical())) {
    return false;
  }
  if ((lhs._threeComponents.firstHorizontal() &&
       !rhs._threeComponents.firstHorizontal()) ||
      (!lhs._threeComponents.firstHorizontal() &&
       rhs._threeComponents.firstHorizontal()) ||
      (lhs._threeComponents.firstHorizontal() &&
       rhs._threeComponents.firstHorizontal() &&
       lhs._threeComponents.firstHorizontal() !=
           rhs._threeComponents.firstHorizontal())) {
    return false;
  }
  if ((lhs._threeComponents.secondHorizontal() &&
       !rhs._threeComponents.secondHorizontal()) ||
      (!lhs._threeComponents.secondHorizontal() &&
       rhs._threeComponents.secondHorizontal()) ||
      (lhs._threeComponents.secondHorizontal() &&
       rhs._threeComponents.secondHorizontal() &&
       lhs._threeComponents.secondHorizontal() !=
           rhs._threeComponents.secondHorizontal())) {
    return false;
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
};

}  // namespace utils
}  // namespace detect
}  // namespace Seiscomp
