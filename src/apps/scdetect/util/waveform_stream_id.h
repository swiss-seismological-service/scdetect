#ifndef SCDETECT_APPS_SCDETECT_UTIL_WAVEFORMSTREAMID_H_
#define SCDETECT_APPS_SCDETECT_UTIL_WAVEFORMSTREAMID_H_

#include <seiscomp/datamodel/waveformstreamid.h>

#include <ostream>
#include <string>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace util {

class WaveformStreamID;
std::string to_string(const WaveformStreamID &waveformStreamId);

void tokenizeWaveformStreamId(const std::string &str,
                              std::vector<std::string> &tokens);

class WaveformStreamID {
 public:
  explicit WaveformStreamID(const std::string &netStaLocCha);
  explicit WaveformStreamID(const DataModel::WaveformStreamID &id);
  WaveformStreamID(const std::string &netCode, const std::string &staCode,
                   const std::string &locCode, const std::string &chaCode);

  static const std::string delimiter;

  // Returns the network code
  const std::string &netCode() const;
  // Returns the station code
  const std::string &staCode() const;
  // Returns the location code
  const std::string &locCode() const;
  // Returns the channel code
  const std::string &chaCode() const;

  // Returns the sensor location stream identifier i.e. in the form
  // `NET.STA.LOC`.
  std::string sensorLocationStreamId() const;

  // Returns `true` if the waveform stream identifier is valid, `false`
  // otherwise.
  bool isValid() const;

  friend std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id);

 private:
  std::string _netCode;
  std::string _staCode;
  std::string _locCode;
  std::string _chaCode;
};

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_UTIL_WAVEFORMSTREAMID_H_
