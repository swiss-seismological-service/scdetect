#ifndef SCDETECT_APPS_CC_UTIL_HORIZONTALCOMPONENTS_H_
#define SCDETECT_APPS_CC_UTIL_HORIZONTALCOMPONENTS_H_

#include <seiscomp/client/inventory.h>
#include <seiscomp/datamodel/utils.h>

#include <cstddef>
#include <string>

namespace Seiscomp {
namespace detect {
namespace util {

class HorizontalComponents {
 public:
  // Load horizontal components identified by `netCode`, `staCode`, `locCode`,
  // `chaCode` and `time` from `inventory`.
  //
  // - It is a bug to pass an invalid pointer as `inventory`.
  // - Throws a `detect::Exception` if loading streams from inventory failed.
  HorizontalComponents(Client::Inventory *inventory, const std::string &netCode,
                       const std::string &staCode, const std::string &locCode,
                       const std::string &chaCode, const Core::Time &time);

  // Allows iteration over the individual streams
  class NEStreamIterator {
   public:
    NEStreamIterator(const HorizontalComponents *horizontalComponents)
        : _horizontalComponents{horizontalComponents} {}

    bool operator!=(const NEStreamIterator &other) const {
      return _pos != other._pos;
    }

    const DataModel::Stream *operator*() const {
      return _horizontalComponents->_threeComponents.comps[_pos];
    }

    const NEStreamIterator &operator++() {
      ++_pos;
      return *this;
    }

   private:
    friend HorizontalComponents;

    std::size_t _pos{1};
    const HorizontalComponents *_horizontalComponents;
  };

  NEStreamIterator begin() const;
  NEStreamIterator end() const;

  // Returns the network code
  const std::string &netCode() const;
  // Returns the station code
  const std::string &staCode() const;
  // Returns the location code
  const std::string &locCode() const;
  // Returns the channel code (i.e. the *band code* and the *source code*
  // identifiers).
  const std::string &chaCode() const;

  friend bool operator==(const HorizontalComponents &lhs,
                         const HorizontalComponents &rhs);
  friend bool operator!=(const HorizontalComponents &lhs,
                         const HorizontalComponents &rhs);

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

// Returns the sensor location stream identifier i.e. in the form
// `NET.STA.LOC`.
//
// - if `includeBandAndSourceCode` is `true` both the band and source code
// identifiers are appended (e.g. `NET.STA.LOC.HH`)
//
// http://docs.fdsn.org/projects/source-identifiers/en/v1.0/channel-codes.html
std::string getSensorLocationStreamId(const HorizontalComponents &c,
                                      bool includeBandAndSourceCode = false);
// Returns a waveform stream identifier omitting the subsource code part.
// I.e. the returned string is of the form `NET.STA.LOC.XY` where `X` refers
// to the *band code* and `Y` refers to the *source code* identifiers.
//
// http://docs.fdsn.org/projects/source-identifiers/en/v1.0/channel-codes.html
std::string getWaveformStreamId(const HorizontalComponents &c);

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_UTIL_THREECOMPONENTS_H_
