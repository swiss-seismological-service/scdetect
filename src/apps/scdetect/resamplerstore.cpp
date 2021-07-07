#include "resamplerstore.h"

#include <boost/functional/hash.hpp>
#include <memory>

#include "utils.h"

namespace std {

inline std::size_t
hash<Seiscomp::detect::record_resampler_store_detail::CacheKey>::operator()(
    const Seiscomp::detect::record_resampler_store_detail::CacheKey &key)
    const noexcept {
  std::size_t ret{0};
  boost::hash_combine(ret, std::hash<double>{}(key.current_frequency));
  boost::hash_combine(ret, std::hash<double>{}(key.target_frequency));
  return ret;
}

}  // namespace std

namespace Seiscomp {
namespace detect {

namespace record_resampler_store_detail {

bool operator==(const CacheKey &lhs, const CacheKey &rhs) {
  return (lhs.current_frequency == rhs.current_frequency &&
          lhs.target_frequency == rhs.target_frequency);
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

void RecordResamplerStore::Reset() { cache_.clear(); }

std::unique_ptr<RecordResamplerStore::RecordResampler>
RecordResamplerStore::Get(const Record *rec, double target_frequency) {
  return Get(rec->samplingFrequency(), target_frequency);
}

std::unique_ptr<RecordResamplerStore::RecordResampler>
RecordResamplerStore::Get(double current_frequency, double target_frequency) {
  record_resampler_store_detail::CacheKey key{current_frequency,
                                              target_frequency};

  if (cache_.find(key) == cache_.end()) {
    cache_.emplace(key,
                   utils::make_unique<RecordResamplerStore::RecordResampler>(
                       target_frequency, fp_, fs_, coefficient_scale_,
                       lanczos_kernel_width_));
  }

  return std::unique_ptr<RecordResamplerStore::RecordResampler>(
      dynamic_cast<RecordResamplerStore::RecordResampler *>(
          cache_.at(key)->clone()));
}

}  // namespace detect
}  // namespace Seiscomp
