#ifndef SCDETECT_APPS_SCDETECT_UTILS_H_
#define SCDETECT_APPS_SCDETECT_UTILS_H_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace utils {

template <typename T> bool IsGeZero(const T num) { return 0 <= num; }
bool ValidatePhase(const std::string &phase);
bool ValidateXCorrThreshold(const double &thres);
bool ValidateStreamID(const std::string &id);

template <typename TMap>
auto map_keys(const TMap &map) -> std::vector<decltype(TMap::key_type)> {
  std::vector<decltype(TMap::key_type)> retval;
  for (const auto &pair : map)
    retval.push_back(pair.first);

  return retval;
}

template <typename TMap>
auto map_values(const TMap &map) -> std::vector<decltype(TMap::key_type)> {
  std::vector<decltype(TMap::key_type)> retval;
  for (const auto &pair : map)
    retval.push_back(pair.second);

  return retval;
}

template <typename TEnum>
auto as_integer(const TEnum value) ->
    typename std::underlying_type<TEnum>::type {
  return static_cast<typename std::underlying_type<TEnum>::type>(value);
}

// Provide C++11 make_unique<T>()
template <typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts &&... params) {
  return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
}

} // namespace utils
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_UTILS_H_
