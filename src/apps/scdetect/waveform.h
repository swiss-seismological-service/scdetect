#ifndef SCDETECT_APPS_SCDETECT_WAVEFORM_H_
#define SCDETECT_APPS_SCDETECT_WAVEFORM_H_

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include <functional>
#include <string>
#include <unordered_map>

#include "exception.h"
#include "version.h"

namespace Seiscomp {
namespace detect {

namespace waveform {

bool trim(GenericRecord &trace, const Core::TimeWindow &tw);
bool filter(GenericRecord &trace, const std::string &filterId);
bool filter(DoubleArray &data, const std::string &filterId,
            double samplingFrequency);
bool resample(GenericRecord &trace, double targetFrequency);
void demean(GenericRecord &trace);
void demean(DoubleArray &data);
void detrend(GenericRecord &trace);
void detrend(DoubleArray &data);
bool write(const GenericRecord &trace, std::ostream &out);
bool read(GenericRecord &trace, std::istream &in);

}  // namespace waveform

DEFINE_SMARTPOINTER(WaveformHandlerIface);
class WaveformHandlerIface : public Core::BaseObject {
 public:
  class BaseException : public Exception {
   public:
    using Exception::Exception;
    BaseException();
  };

  struct ProcessingConfig {
    // The filter identifier
    std::string filterId;
    // Margin time in seconds used for filtering in order to prevent from
    // filtering artifacts
    double filterMarginTime{0};
    // Target frequency for resampling
    double targetFrequency{0};
    // Indicates if the data should be demeaned
    bool demean{true};
  };

  virtual GenericRecordCPtr get(const DataModel::WaveformStreamID &id,
                                const Core::TimeWindow &tw,
                                const ProcessingConfig &config) = 0;

  virtual GenericRecordCPtr get(const std::string &netCode,
                                const std::string &staCode,
                                const std::string &locCode,
                                const std::string &chaCode,
                                const Core::TimeWindow &tw,
                                const ProcessingConfig &config) = 0;

  virtual GenericRecordCPtr get(const DataModel::WaveformStreamID &id,
                                const Core::Time &start, const Core::Time &end,
                                const ProcessingConfig &config) = 0;

  virtual GenericRecordCPtr get(const std::string &netCode,
                                const std::string &staCode,
                                const std::string &locCode,
                                const std::string &chaCode,
                                const Core::Time &start, const Core::Time &end,
                                const ProcessingConfig &config) = 0;

 protected:
  // Process `trace` according to `config`
  void process(const GenericRecordPtr &trace, const ProcessingConfig &config,
               const Core::TimeWindow &twTrim = Core::TimeWindow{}) const;
};

DEFINE_SMARTPOINTER(WaveformHandler);
class WaveformHandler : public WaveformHandlerIface {
 public:
  class NoData : public BaseException {
   public:
    using BaseException::BaseException;
    NoData();
  };

  explicit WaveformHandler(const std::string &recordStreamUrl);

  GenericRecordCPtr get(
      const DataModel::WaveformStreamID &id, const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr get(
      const std::string &netCode, const std::string &staCode,
      const std::string &locCode, const std::string &chaCode,
      const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr get(
      const DataModel::WaveformStreamID &id, const Core::Time &start,
      const Core::Time &end,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr get(
      const std::string &netCode, const std::string &staCode,
      const std::string &locCode, const std::string &chaCode,
      const Core::Time &start, const Core::Time &end,
      const WaveformHandlerIface::ProcessingConfig &config) override;

 private:
  std::string _recordStreamUrl;

  static const double _downloadMargin;
};

DEFINE_SMARTPOINTER(Cached);
class Cached : public WaveformHandlerIface {
 public:
  GenericRecordCPtr get(
      const DataModel::WaveformStreamID &id, const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr get(
      const std::string &cacheKey, const std::string &staCode,
      const std::string &locCode, const std::string &chaCode,
      const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr get(
      const DataModel::WaveformStreamID &id, const Core::Time &start,
      const Core::Time &end,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr get(
      const std::string &netCode, const std::string &staCode,
      const std::string &locCode, const std::string &chaCode,
      const Core::Time &start, const Core::Time &end,
      const WaveformHandlerIface::ProcessingConfig &config) override;

 protected:
  explicit Cached(WaveformHandlerIfacePtr waveformHandler, bool raw = false);

  virtual void makeCacheKey(
      const std::string &netCode, const std::string &staCode,
      const std::string &locCode, const std::string &chaCode,
      const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config,
      std::string &result) const;
  virtual void makeCacheKey(std::vector<std::string> keyComponents,
                            std::string &result) const;
  virtual GenericRecordCPtr get(const std::string &key) = 0;
  virtual bool set(const std::string &key, GenericRecordCPtr value) = 0;
  virtual bool exists(const std::string &key) = 0;

  virtual bool cacheProcessed() const;

 private:
  WaveformHandlerIfacePtr _waveformHandler;

  // Indicates if either the raw waveform or the processed waveform should be
  // cached
  bool _raw;

  static const std::string _cacheKeySep;
};

DEFINE_SMARTPOINTER(FileSystemCache);
class FileSystemCache : public Cached {
 public:
  FileSystemCache(WaveformHandlerIfacePtr waveform_handler,
                  const std::string &path, bool raw = false);

 protected:
  GenericRecordCPtr get(const std::string &key) override;
  bool set(const std::string &key, GenericRecordCPtr value) override;
  bool exists(const std::string &key) override;

 private:
  std::string _pathCache;
};

DEFINE_SMARTPOINTER(InMemoryCache);
class InMemoryCache : public Cached {
 public:
  explicit InMemoryCache(WaveformHandlerIfacePtr waveformHandler,
                         bool raw = false);

 protected:
  GenericRecordCPtr get(const std::string &key) override;
  bool set(const std::string &key, GenericRecordCPtr value) override;
  bool exists(const std::string &key) override;

 private:
  std::unordered_map<std::string, GenericRecordCPtr> _cache;
};

}  // namespace detect
}  // namespace Seiscomp

namespace std {

template <>
struct hash<Seiscomp::detect::WaveformHandlerIface::ProcessingConfig> {
  std::size_t operator()(
      const Seiscomp::detect::WaveformHandlerIface::ProcessingConfig &c)
      const noexcept;
};

}  // namespace std

#endif  // SCDETECT_APPS_SCDETECT_WAVEFORM_H_
