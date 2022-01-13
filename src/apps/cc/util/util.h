#ifndef SCDETECT_APPS_CC_UTIL_UTIL_H_
#define SCDETECT_APPS_CC_UTIL_UTIL_H_

#include <boost/filesystem/path.hpp>
#include <string>
#include <type_traits>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace util {

const std::string createUUID();
void replaceEscapedXMLFilterIdChars(std::string &filter_id);
// Safely create the directory path `p`
bool createDirectory(const boost::filesystem::path &p);

template <typename T>
bool isGeZero(const T num) {
  return 0 <= num;
}

template <typename TMap>
auto map_keys(const TMap &map) -> std::vector<typename TMap::key_type> {
  std::vector<typename TMap::key_type> retval;
  for (const auto &pair : map) retval.push_back(pair.first);

  return retval;
}

template <typename TMap>
auto map_values(const TMap &map) -> std::vector<typename TMap::mapped_type> {
  std::vector<typename TMap::mapped_type> retval;
  for (const auto &pair : map) retval.push_back(pair.second);

  return retval;
}

template <typename TEnum>
auto asInteger(const TEnum value) ->
    typename std::underlying_type<TEnum>::type {
  return static_cast<typename std::underlying_type<TEnum>::type>(value);
}

template <typename TMap, typename Predicate>
std::vector<typename TMap::key_type> filterKeys(const TMap &m, Predicate &p) {
  std::vector<typename TMap::key_type> retval;
  for (const auto &pair : m) {
    if (p(pair)) {
      retval.push_back(pair.first);
    }
  }
  return retval;
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_UTIL_UTIL_H_
