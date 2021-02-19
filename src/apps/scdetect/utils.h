#ifndef SCDETECT_APPS_SCDETECT_UTILS_H_
#define SCDETECT_APPS_SCDETECT_UTILS_H_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <seiscomp/core/defs.h>
#include <seiscomp/datamodel/waveformstreamid.h>

namespace Seiscomp {
namespace detect {
namespace utils {

const std::string CreateUUID();

template <typename T> bool IsGeZero(const T num) { return 0 <= num; }
bool ValidateXCorrThreshold(const double &thres);

template <typename TMap>
auto map_keys(const TMap &map) -> std::vector<typename TMap::key_type> {
  std::vector<typename TMap::key_type> retval;
  for (const auto &pair : map)
    retval.push_back(pair.first);

  return retval;
}

template <typename TMap>
auto map_values(const TMap &map) -> std::vector<typename TMap::mapped_type> {
  std::vector<typename TMap::mapped_type> retval;
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

template <typename T, typename... Ts>
typename Core::SmartPointer<T>::Impl make_smart(Ts &&... params) {
  return
      typename Core::SmartPointer<T>::Impl(new T(std::forward<Ts>(params)...));
}

template <typename TMap, typename Predicate>
std::vector<typename TMap::key_type> filter_keys(const TMap &m, Predicate &p) {

  std::vector<typename TMap::key_type> retval;
  for (const auto &pair : m) {
    if (p(pair)) {
      retval.push_back(pair.first);
    }
  }
  return retval;
}

// Compute the mean value of `samples` using a cumulative moving average
// algorithm.
template <typename T> double CMA(T *samples, size_t n) {
  double cma{0};
  // cummulative moving average for samples a_0, ..., a_n:
  //
  // mean_n = mean_{n-1} + (a_n - mean_{n-1}) / n
  //
  for (size_t i = 0; i < n; ++i) {
    cma += (samples[i] - cma) / (i + 1);
  }
  return cma;
}

// Returns `true` if the difference between two floating point numbers is
// smaller than epsilon, else `false`
template <typename TFloatingPoint>
bool AlmostEqual(TFloatingPoint lhs, TFloatingPoint rhs,
                 TFloatingPoint epsilon) {
  // The IEEE standard says that any comparison operation involving
  // a NAN must return false.
  if (isnan(lhs) || isnan(rhs)) {
    return false;
  }

  // From Knuth - The Art of Computer Programming
  return std::abs(rhs - lhs) <=
         std::max(std::abs(lhs), std::abs(rhs)) * epsilon;
}

// Returns `true` if `lhs` is greater than `rhs` under consideration of an
// accuracy of `epsilon`. If `lhs` is smaller than `rhs`, `false` is returned.
template <typename TFloatingPoint>
bool GreaterThan(TFloatingPoint lhs, TFloatingPoint rhs,
                 TFloatingPoint epsilon) {
  // The IEEE standard says that any comparison operation involving
  // a NAN must return false.
  if (isnan(lhs) || isnan(rhs)) {
    return false;
  }

  // From Knuth - The Art of Computer Programming
  return (lhs - rhs) > std::max(std::abs(lhs), std::abs(rhs)) * epsilon;
}

// Returns `true` if `lhs` is smaller than `rhs` under consideration of an
// accuracy of `epsilon`. If `lhs` is smaller than `rhs`, `false` is returned.
template <typename TFloatingPoint>
bool LessThan(TFloatingPoint lhs, TFloatingPoint rhs, TFloatingPoint epsilon) {
  // The IEEE standard says that any comparison operation involving
  // a NAN must return false.
  if (isnan(lhs) || isnan(rhs)) {
    return false;
  }

  // From Knuth - The Art of Computer Programming
  return (rhs - lhs) > std::max(std::abs(lhs), std::abs(rhs)) * epsilon;
}

/* ------------------------------------------------------------------------- */
class WaveformStreamID {
public:
  explicit WaveformStreamID(const std::string &net_sta_loc_cha);
  explicit WaveformStreamID(const DataModel::WaveformStreamID &id);
  WaveformStreamID(const std::string &net_code, const std::string &sta_code,
                   const std::string &loc_code, const std::string &cha_code);

  const std::string &net_code() const;
  const std::string &sta_code() const;
  const std::string &loc_code() const;
  const std::string &cha_code() const;

  bool IsValid() const;

  friend std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id);

protected:
  const std::string delimiter_{"."};

private:
  std::string net_code_;
  std::string sta_code_;
  std::string loc_code_;
  std::string cha_code_;
};

} // namespace utils
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_UTILS_H_
