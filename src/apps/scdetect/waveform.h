#ifndef SCDETECT_APPS_SCDETECT_WAVEFORM_MANAGER_H_
#define SCDETECT_APPS_SCDETECT_WAVEFORM_MANAGER_H_

#include <string>
#include <unordered_map>

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/typedarray.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include "exception.h"
#include "version.h"

namespace Seiscomp {
namespace detect {

namespace waveform {

bool Merge(GenericRecord &trace, const RecordSequence &seq);
bool Trim(GenericRecord &trace, const Core::TimeWindow &tw);
bool Filter(GenericRecord &trace, const std::string &filter_string);
void Resample(GenericRecord &trace, double sampling_frequency);
void Resample(DoubleArrayPtr data, double sampling_frequency_from,
              double sampling_frequency_to);
void Demean(GenericRecord &trace);
void Demean(DoubleArrayPtr data);
bool Write(const GenericRecord &trace, std::ostream &out);
bool Read(GenericRecord &trace, std::istream &in);

} // namespace waveform

DEFINE_SMARTPOINTER(WaveformHandlerIface);
class WaveformHandlerIface : public Core::BaseObject {
public:
  class BaseException : public Exception {
  public:
    using Exception::Exception;
    BaseException();
  };

  struct ProcessingConfig {
    std::string filter_string{""};
    double resample_frequency{0};
    bool demean{true};
  };

  virtual ~WaveformHandlerIface() {}

  virtual GenericRecordCPtr Get(const DataModel::WaveformStreamID &id,
                                const Core::TimeWindow &tw,
                                const ProcessingConfig &config) = 0;

  virtual GenericRecordCPtr
  Get(const std::string &net_code, const std::string &sta_code,
      const std::string &loc_code, const std::string &cha_code,
      const Core::TimeWindow &tw, const ProcessingConfig &config) = 0;

  virtual GenericRecordCPtr Get(const DataModel::WaveformStreamID &id,
                                const Core::Time &start, const Core::Time &end,
                                const ProcessingConfig &config) = 0;

  virtual GenericRecordCPtr Get(const std::string &net_code,
                                const std::string &sta_code,
                                const std::string &loc_code,
                                const std::string &cha_code,
                                const Core::Time &start, const Core::Time &end,
                                const ProcessingConfig &config) = 0;
};

DEFINE_SMARTPOINTER(WaveformHandler);
class WaveformHandler : public WaveformHandlerIface {
public:
  class NoData : public BaseException {
  public:
    using BaseException::BaseException;
    NoData();
  };

  WaveformHandler(const std::string &record_stream_url);

  GenericRecordCPtr
  Get(const DataModel::WaveformStreamID &id, const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr
  Get(const std::string &net_code, const std::string &sta_code,
      const std::string &loc_code, const std::string &cha_code,
      const Core::TimeWindow &tw,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr
  Get(const DataModel::WaveformStreamID &id, const Core::Time &start,
      const Core::Time &end,
      const WaveformHandlerIface::ProcessingConfig &config) override;

  GenericRecordCPtr
  Get(const std::string &net_code, const std::string &sta_code,
      const std::string &loc_code, const std::string &cha_code,
      const Core::Time &start, const Core::Time &end,
      const WaveformHandlerIface::ProcessingConfig &config) override;

private:
  std::string record_stream_url_;

  double download_margin_{2};
};

DEFINE_SMARTPOINTER(Cached);
class Cached : public WaveformHandlerIface {
public:
  GenericRecordCPtr Get(const DataModel::WaveformStreamID &id,
                        const Core::TimeWindow &tw,
                        const WaveformHandlerIface::ProcessingConfig &config);

  GenericRecordCPtr Get(const std::string &net_code,
                        const std::string &sta_code,
                        const std::string &loc_code,
                        const std::string &cha_code, const Core::TimeWindow &tw,
                        const WaveformHandlerIface::ProcessingConfig &config);

  GenericRecordCPtr Get(const DataModel::WaveformStreamID &id,
                        const Core::Time &start, const Core::Time &end,
                        const WaveformHandlerIface::ProcessingConfig &config);

  GenericRecordCPtr Get(const std::string &net_code,
                        const std::string &sta_code,
                        const std::string &loc_code,
                        const std::string &cha_code, const Core::Time &start,
                        const Core::Time &end,
                        const WaveformHandlerIface::ProcessingConfig &config);

protected:
  Cached(WaveformHandlerIfacePtr waveform_handler, bool raw = false);

  virtual void
  MakeCacheKey(const std::string &net_code, const std::string &sta_code,
               const std::string &loc_code, const std::string &cha_code,
               const Core::TimeWindow &tw,
               const WaveformHandlerIface::ProcessingConfig &config,
               std::string &result);
  virtual void MakeCacheKey(std::vector<std::string> key_components,
                            std::string &result);
  virtual GenericRecordCPtr Get(const std::string &key) = 0;
  virtual bool Set(const std::string &key, GenericRecordCPtr value) = 0;
  virtual bool Exists(const std::string &key) = 0;

  virtual bool CacheProcessed() const;

private:
  WaveformHandlerIfacePtr waveform_handler_;

  // Idicates if either the raw waveform or the processed waveform should be
  // cached
  bool raw_;

  const std::string cache_key_sep_{"."};
};

DEFINE_SMARTPOINTER(FileSystemCache);
class FileSystemCache : public Cached {
public:
  FileSystemCache(WaveformHandlerIfacePtr waveform_handler,
                  const std::string &path, bool raw = false);

protected:
  GenericRecordCPtr Get(const std::string &key) override;
  bool Set(const std::string &key, GenericRecordCPtr value) override;
  bool Exists(const std::string &key) override;

private:
  std::string path_cache_;
};

DEFINE_SMARTPOINTER(InMemoryCache);
class InMemoryCache : public Cached {
public:
  InMemoryCache(WaveformHandlerIfacePtr waveform_handler, bool raw = false);

protected:
  GenericRecordCPtr Get(const std::string &key) override;
  bool Set(const std::string &key, GenericRecordCPtr value) override;
  bool Exists(const std::string &key) override;

private:
  std::unordered_map<std::string, GenericRecordCPtr> cache_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_WAVEFORM_MANAGER_H_
