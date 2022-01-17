#ifndef SCDETECT_APPS_SCDETECT_RESAMPLERSTORE_H_
#define SCDETECT_APPS_SCDETECT_RESAMPLERSTORE_H_

#include <seiscomp/io/recordfilter/resample.h>

#include <functional>
#include <memory>
#include <unordered_map>

namespace Seiscomp {
namespace detect {
namespace record_resampler_store_detail {

struct CacheKey {
  double currentFrequency;
  double targetFrequency;

  friend bool operator==(const CacheKey &lhs, const CacheKey &rhs);
  friend bool operator!=(const CacheKey &lhs, const CacheKey &rhs);
};

}  // namespace record_resampler_store_detail
}  // namespace detect
}  // namespace Seiscomp

namespace std {
template <>
struct hash<Seiscomp::detect::record_resampler_store_detail::CacheKey> {
  std::size_t operator()(
      const Seiscomp::detect::record_resampler_store_detail::CacheKey &key)
      const noexcept;
};

}  // namespace std

namespace Seiscomp {
namespace detect {

// A global store for resamplers
// - implements the Singleton Design Pattern
class RecordResamplerStore {
 public:
  using RecordResampler = IO::RecordResampler<double>;
  static RecordResamplerStore &Instance();

  RecordResamplerStore(const RecordResamplerStore &) = delete;
  RecordResamplerStore &operator=(const RecordResamplerStore &) = delete;

  // Reset the store
  void reset();

  std::unique_ptr<RecordResampler> get(const Record *rec,
                                       double targetFrequency);

  std::unique_ptr<RecordResampler> get(double currentFrequency,
                                       double targetFrequency);

 private:
  RecordResamplerStore() {}

  struct CacheKey {
    double sourceFrequency;
    double targetFrequency;
  };

  using Cache = std::unordered_map<record_resampler_store_detail::CacheKey,
                                   std::unique_ptr<RecordResampler>>;

  Cache _cache;

  double _fp{0.7};
  double _fs{0.9};
  double _coefficientScale{10};
  int _lanczosKernelWidth{3};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_RESAMPLERSTORE_H_
