#ifndef SCDETECT_APPS_SCDETECT_UTIL_WAVEFORMSTREAMID_H_
#define SCDETECT_APPS_SCDETECT_UTIL_WAVEFORMSTREAMID_H_

#include <seiscomp/datamodel/waveformstreamid.h>

#include <ostream>
#include <string>

namespace Seiscomp {
namespace detect {
namespace util {

class WaveformStreamID;
std::string to_string(const WaveformStreamID &waveformStreamId);

class WaveformStreamID {
 public:
  explicit WaveformStreamID(const std::string &netStaLocCha);
  explicit WaveformStreamID(const DataModel::WaveformStreamID &id);
  WaveformStreamID(const std::string &netCode, const std::string &staCode,
                   const std::string &locCode, const std::string &chaCode);

  // Returns the network code
  const std::string &netCode() const;
  // Returns the station code
  const std::string &staCode() const;
  // Returns the location code
  const std::string &locCode() const;
  // Returns the channel code
  const std::string &chaCode() const;

  // Returns the sensor location stream identifier i.e. in the form
  // `NET.STA.LOC.`.
  std::string sensorLocationStreamId() const;

  // Returns `true` if the waveform stream identifier is valid, `false`
  // otherwise.
  bool isValid() const;

  friend std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id);

 protected:
  static const std::string _delimiter;

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
