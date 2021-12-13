#include "resamplerstore.h"

#include <boost/functional/hash.hpp>
#include <memory>

#include "util/memory.h"

namespace std {

inline std::size_t
hash<Seiscomp::detect::record_resampler_store_detail::CacheKey>::operator()(
    const Seiscomp::detect::record_resampler_store_detail::CacheKey &key)
    const noexcept {
  std::size_t ret{0};
  boost::hash_combine(ret, std::hash<double>{}(key.currentFrequency));
  boost::hash_combine(ret, std::hash<double>{}(key.targetFrequency));
  return ret;
}

}  // namespace std

namespace Seiscomp {
namespace detect {

namespace record_resampler_store_detail {

bool operator==(const CacheKey &lhs, const CacheKey &rhs) {
  return (lhs.currentFrequency == rhs.currentFrequency &&
          lhs.targetFrequency == rhs.targetFrequency);
}

bool operator!=(const CacheKey &lhs, const CacheKey &rhs) {
  return !(lhs == rhs);
}

}  // namespace record_resampler_store_detail

RecordResamplerStore &RecordResamplerStore::Instance() {
  // guaranteed to be destroyed; instantiated on first use
  static RecordResamplerStore instance;
  return instance;
}

void RecordResamplerStore::reset() { _cache.clear(); }

std::unique_ptr<RecordResamplerStore::RecordResampler>
RecordResamplerStore::get(const Record *rec, double targetFrequency) {
  return get(rec->samplingFrequency(), targetFrequency);
}

std::unique_ptr<RecordResamplerStore::RecordResampler>
RecordResamplerStore::get(double currentFrequency, double targetFrequency) {
  record_resampler_store_detail::CacheKey key{currentFrequency,
                                              targetFrequency};

  if (_cache.find(key) == _cache.end()) {
    _cache.emplace(
        key,
        util::make_unique<RecordResamplerStore::RecordResampler>(
            targetFrequency, _fp, _fs, _coefficientScale, _lanczosKernelWidth));
  }

  return std::unique_ptr<RecordResamplerStore::RecordResampler>(
      dynamic_cast<RecordResamplerStore::RecordResampler *>(
          _cache.at(key)->clone()));
}

}  // namespace detect
}  // namespace Seiscomp
