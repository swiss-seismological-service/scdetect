#ifndef SCDETECT_APPS_SCDETECT_RESAMPLERSTORE_H_
#define SCDETECT_APPS_SCDETECT_RESAMPLERSTORE_H_

#include <functional>
#include <memory>
#include <unordered_map>

#include <seiscomp/io/recordfilter/resample.h>

namespace Seiscomp {
namespace detect {
namespace record_resampler_store_detail {

struct CacheKey {
  double current_frequency;
  double target_frequency;

  friend bool operator==(const CacheKey &lhs, const CacheKey &rhs);
  friend bool operator!=(const CacheKey &lhs, const CacheKey &rhs);
};

} // namespace record_resampler_store_detail
} // namespace detect
} // namespace Seiscomp

namespace std {
template <>
struct hash<Seiscomp::detect::record_resampler_store_detail::CacheKey> {
  std::size_t operator()(
      const Seiscomp::detect::record_resampler_store_detail::CacheKey &key)
      const noexcept;
};

} // namespace std

namespace Seiscomp {
namespace detect {

// A global store for resamplers
// - implements the Singleton Design Pattern
class RecordResamplerStore {
  using RecordResampler = IO::RecordResampler<double>;

public:
  static RecordResamplerStore &Instance();

  RecordResamplerStore(const RecordResamplerStore &) = delete;
  void operator=(const RecordResamplerStore &) = delete;

  // Reset the store
  void Reset();

  std::unique_ptr<RecordResampler> Get(const Record *rec,
                                       double target_frequency);

  std::unique_ptr<RecordResampler> Get(double current_frequency,
                                       double target_frequency);

private:
  RecordResamplerStore() {}

  struct CacheKey {
    double source_frequency;
    double target_frequency;
  };

  using Cache = std::unordered_map<record_resampler_store_detail::CacheKey,
                                   std::unique_ptr<RecordResampler>>;

  Cache cache_;

  double fp_{0.7};
  double fs_{0.9};
  double coefficient_scale_{10};
  int lanczos_kernel_width_{3};
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_RESAMPLERSTORE_H_
