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

std::string join(const std::string &netCode, const std::string &staCode,
                 const std::string &locCode, const std::string &chaCode = "");

// Returns the sensor location stream identifier i.e. in the form
// `NET.STA.LOC`.
//
// - if `includeBandAndSourceCode` is `true` both the band and source code
// identifiers are appended (e.g. `NET.STA.LOC.HH`)
//
// http://docs.fdsn.org/projects/source-identifiers/en/v1.0/channel-codes.html
std::string getSensorLocationStreamId(const WaveformStreamID &waveformStreamId,
                                      bool includeBandAndSourceCode = false);

std::string getSensorLocationStreamId(const std::string &waveformStreamId,
                                      bool includeBandAndSourceCode = false);

std::string getBandAndSourceCode(const WaveformStreamID &waveformStreamId);

std::string getBandAndSourceCode(const std::string &chaCode);

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

  friend std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id);

 private:
  // Returns `true` if the waveform stream identifier is valid, `false`
  // otherwise.
  bool isValid() const;

  std::string _netCode;
  std::string _staCode;
  std::string _locCode;
  std::string _chaCode;
};

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_UTIL_WAVEFORMSTREAMID_H_
