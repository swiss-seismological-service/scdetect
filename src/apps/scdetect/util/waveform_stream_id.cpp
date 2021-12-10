#include "waveform_stream_id.h"

#include <seiscomp/core/strings.h>

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

std::string getSensorLocationStreamId(const WaveformStreamID &waveformId,
                                      bool includeBandAndSourceCode) {
  return waveformId.netCode() + WaveformStreamID::delimiter +
         waveformId.staCode() + WaveformStreamID::delimiter +
         waveformId.locCode() +
         (includeBandAndSourceCode
              ? WaveformStreamID::delimiter + waveformId.chaCode()
              : "");
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

std::string WaveformStreamID::sensorLocationStreamId() const {
  return _netCode + delimiter + _staCode + delimiter + _locCode;
}

bool WaveformStreamID::isValid() const {
  return !(_netCode.empty() || _staCode.empty() || _chaCode.empty());
}

std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id) {
  os << id._netCode << id.delimiter << id._staCode << id.delimiter
     << id._locCode << id.delimiter << id._chaCode;
  return os;
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
