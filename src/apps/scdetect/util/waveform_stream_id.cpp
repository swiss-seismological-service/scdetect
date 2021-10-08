#include "waveform_stream_id.h"

#include <seiscomp/core/strings.h>

#include "../exception.h"
#include "../settings.h"

namespace Seiscomp {
namespace detect {
namespace util {

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

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
