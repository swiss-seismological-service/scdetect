#ifndef SCDETECT_APPS_SCDETECT_UTIL_THREECOMPONENTS_H_
#define SCDETECT_APPS_SCDETECT_UTIL_THREECOMPONENTS_H_

#include <seiscomp/client/inventory.h>
#include <seiscomp/datamodel/utils.h>

#include <string>
#include <vector>

#include "waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace util {
// Wraps `DataModel::ThreeComponents`
//
// - TODO(damb): implement an iterator in order to iterate over the components
class ThreeComponents {
 public:
  // Load components identified by `netCode`, `staCode`, `locCode`, `chaCode`
  // and `time` from `inventory`.
  //
  // - It is a bug to pass an invalid pointer as `inventory`.
  // - Throws a `detect::Exception` if loading streams from inventory failed.
  ThreeComponents(Client::Inventory *inventory, const std::string &netCode,
                  const std::string &staCode, const std::string &locCode,
                  const std::string &chaCode, const Core::Time &time);

  // Returns the network code
  const std::string &netCode() const;
  // Returns the station code
  const std::string &staCode() const;
  // Returns the location code
  const std::string &locCode() const;
  // Returns the channel code (i.e. the *band code* and the *source code*
  // identifiers).
  const std::string &chaCode() const;

  // Returns the sensor location stream identifier i.e. in the form
  // `NET.STA.LOC`.
  //
  // http://docs.fdsn.org/projects/source-identifiers/en/v1.0/channel-codes.html
  std::string sensorLocationStreamId() const;
  // Returns the stream code identifiers for all components
  std::vector<std::string> streamCodes() const;
  // Returns the waveform stream identifiers for all components
  std::vector<util::WaveformStreamID> waveformStreamIds() const;
  // Returns a waveform stream identifier omitting the subsource code part.
  // I.e. the returned string is of the form `NET.STA.LOC.XY` where `X` refers
  // to the *band code* and `Y` refers to the *source code* identifiers.
  //
  // http://docs.fdsn.org/projects/source-identifiers/en/v1.0/channel-codes.html
  std::string waveformStreamId() const;

  // Returns a reference to the underlying `DataModel::ThreeComponents` object
  const DataModel::ThreeComponents &threeComponents() const;

  friend bool operator==(const ThreeComponents &lhs,
                         const ThreeComponents &rhs);
  friend bool operator!=(const ThreeComponents &lhs,
                         const ThreeComponents &rhs);

 protected:
  // Returns the *real size* i.e. the number of components actually available
  size_t realSize() const;
  // Reset all components
  void reset();

 private:
  std::string _networkCode;
  std::string _stationCode;
  std::string _locationCode;
  std::string _channelCode;

  DataModel::ThreeComponents _threeComponents;
};

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_UTIL_THREECOMPONENTS_H_
