#include "waveform_stream_id.h"

#include <seiscomp/core/strings.h>

#include <cassert>
#include <unordered_set>

#include "../exception.h"

namespace Seiscomp {
namespace detect {
namespace util {

std::string to_string(const WaveformStreamID &waveformStreamId) {
  std::ostringstream oss;
  oss << waveformStreamId;
  return oss.str();
}

void tokenizeWaveformStreamId(const std::string &str,
                              std::vector<std::string> &tokens) {
  Core::split(tokens, str, WaveformStreamID::delimiter.c_str(), false);
}

std::string join(const std::string &netCode, const std::string &staCode,
                 const std::string &locCode, const std::string &chaCode) {
  return netCode + WaveformStreamID::delimiter + staCode +
         WaveformStreamID::delimiter + locCode +
         (chaCode.empty() ? "" : WaveformStreamID::delimiter + chaCode);
}

std::string getBandAndSourceCode(const WaveformStreamID &waveformStreamId) {
  return getBandAndSourceCode(waveformStreamId.chaCode());
}

std::string getBandAndSourceCode(const std::string &chaCode) {
  return chaCode.substr(0, 2);
}

std::string getSensorLocationStreamId(const WaveformStreamID &waveformStreamId,
                                      bool includeBandAndSourceCode) {
  return waveformStreamId.netCode() + WaveformStreamID::delimiter +
         waveformStreamId.staCode() + WaveformStreamID::delimiter +
         waveformStreamId.locCode() +
         (includeBandAndSourceCode ? WaveformStreamID::delimiter +
                                         getBandAndSourceCode(waveformStreamId)
                                   : "");
}

std::string getSensorLocationStreamId(const std::string &waveformStreamId,
                                      bool includeBandAndSourceCode) {
  std::vector<std::string> tokens;
  tokenizeWaveformStreamId(waveformStreamId, tokens);
  // XXX(damb): do not validate the token content
  assert(((includeBandAndSourceCode && tokens.size() == 4) ||
          (!includeBandAndSourceCode && tokens.size() == 3)));
  return tokens[0] + WaveformStreamID::delimiter + tokens[1] +
         WaveformStreamID::delimiter + tokens[2] +
         (includeBandAndSourceCode
              ? (WaveformStreamID::delimiter + tokens[3].substr(0, 2))
              : "");
}

bool isUniqueSensorLocation(const std::vector<std::string> &waveformStreamIds,
                            bool includeBandAndSourceCode) {
  std::unordered_set<std::string> sensorLocationStreamIds;
  for (const auto &waveformStreamId : waveformStreamIds) {
    try {
      sensorLocationStreamIds.emplace(getSensorLocationStreamId(
          WaveformStreamID(waveformStreamId), includeBandAndSourceCode));
    } catch (const ValueException &) {
      return false;
    }
  }
  return sensorLocationStreamIds.size() == 1;
}

const std::string WaveformStreamID::delimiter{"."};

WaveformStreamID::WaveformStreamID(const std::string &netStaLocCha) {
  std::vector<std::string> tokens;
  tokenizeWaveformStreamId(netStaLocCha, tokens);

  if (4 != tokens.size()) {
    throw ValueException{std::string{"invalid number of tokens: "} +
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
    throw ValueException{std::string{"invalid DataModel::WaveformStreamID: "} +
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

std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id) {
  os << id._netCode << WaveformStreamID::delimiter << id._staCode
     << WaveformStreamID::delimiter << id._locCode
     << WaveformStreamID::delimiter << id._chaCode;
  return os;
}

bool operator==(const WaveformStreamID &lhs, const WaveformStreamID &rhs) {
  return std::tie(lhs.netCode(), lhs.staCode(), lhs.locCode(), lhs.chaCode()) ==
         std::tie(rhs.netCode(), rhs.staCode(), rhs.locCode(), rhs.chaCode());
}

bool operator!=(const WaveformStreamID &lhs, const WaveformStreamID &rhs) {
  return !(lhs == rhs);
}

bool operator<(const WaveformStreamID &lhs, const WaveformStreamID &rhs) {
  return std::tie(lhs.netCode(), lhs.staCode(), lhs.locCode(), lhs.chaCode()) <
         std::tie(rhs.netCode(), rhs.staCode(), rhs.locCode(), rhs.chaCode());
}

bool operator>(const WaveformStreamID &lhs, const WaveformStreamID &rhs) {
  return rhs < lhs;
}

bool operator<=(const WaveformStreamID &lhs, const WaveformStreamID &rhs) {
  return !(lhs > rhs);
}

bool operator>=(const WaveformStreamID &lhs, const WaveformStreamID &rhs) {
  return !(lhs < rhs);
}

bool WaveformStreamID::isValid() const {
  return !(_netCode.empty() || _staCode.empty() || _chaCode.empty());
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
