#include "waveform.h"

#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/strings.h>
#include <seiscomp/io/recordinput.h>
#include <seiscomp/io/records/mseedrecord.h>
#include <seiscomp/io/recordstream.h>
#include <seiscomp/math/filter.h>
#include <seiscomp/utils/files.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>
#include <fstream>
#include <memory>

#include "log.h"
#include "resamplerstore.h"
#include "util/math.h"
#include "util/memory.h"
#include "util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {
namespace waveform {

namespace {

template <class T>
T nextPowerOfTwo(T a, T min = 1, T max = 1 << 31) {
  int b = min;
  while (b < a) {
    b <<= 1;
    if (b > max) return -1;
  }
  return b;
}

}  // namespace

bool trim(GenericRecord &trace, const Core::TimeWindow &tw) {
  if (trace.timeWindow() == tw) {
    return true;
  }

  auto beginOffset{static_cast<int>(
      std::floor(static_cast<double>(tw.startTime() - trace.startTime()) *
                 trace.samplingFrequency()))};
  auto endOffset{static_cast<int>(
      std::ceil(static_cast<double>(tw.endTime() - trace.startTime()) *
                trace.samplingFrequency()))};

  // one sample tolerance
  if (beginOffset == -1) {
    ++beginOffset;
  }
  if (endOffset == trace.data()->size() + 1) {
    --endOffset;
  }

  // not enough data at start of time window
  if (beginOffset < 0) {
    return false;
  }
  // not enough data at end of time window
  if (endOffset > trace.data()->size()) {
    return false;
  }

  ArrayPtr sliced{trace.data()->slice(beginOffset, endOffset)};
  if (!sliced) {
    return false;
  }
  trace.setData(sliced.get());
  trace.setStartTime(trace.startTime() +
                     Core::TimeSpan{beginOffset / trace.samplingFrequency()});
  return true;
}

bool filter(GenericRecord &trace, const std::string &filterId) {
  if (filterId.empty()) return false;

  auto data{DoubleArray::Cast(trace.data())};
  if (!filter(*data, filterId, trace.samplingFrequency())) {
    return false;
  }
  trace.dataUpdated();

  return true;
}

bool filter(DoubleArray &data, const std::string &filterId,
            double samplingFrequency) {
  if (filterId.empty() || samplingFrequency <= 0) return false;

  std::string filterError;
  auto filter =
      Math::Filtering::InPlaceFilter<double>::Create(filterId, &filterError);
  if (!filter) {
    SCDETECT_LOG_WARNING("Filter creation failed for '%s': %s",
                         filterId.c_str(), filterError.c_str());
    return false;
  }
  filter->setSamplingFrequency(samplingFrequency);
  filter->apply(data.size(), data.typedData());
  delete filter;

  return true;
}

bool resample(GenericRecord &trace, double targetFrequency) {
  if (targetFrequency <= 0 || trace.samplingFrequency() == targetFrequency)
    return true;

  auto resampler{RecordResamplerStore::Instance().get(&trace, targetFrequency)};
  std::unique_ptr<Record> resampled;
  resampled.reset(resampler->feed(&trace));
  if (!resampled) {
    SCDETECT_LOG_WARNING(
        "%s: Failed to resample record "
        "(samplingFrequency=%f): targetFrequency=%f",
        std::string{trace.streamID()}.c_str(), trace.samplingFrequency(),
        targetFrequency);
    return false;
  }

  trace.setStartTime(resampled->startTime());
  trace.setSamplingFrequency(static_cast<double>(targetFrequency));
  trace.setData(resampled->data()->copy(Array::DataType::DOUBLE));
  return true;
}

void demean(GenericRecord &trace) {
  auto data{DoubleArray::Cast(trace.data())};
  demean(*data);
  trace.dataUpdated();
}

void demean(DoubleArray &data) {
  const auto mean{util::cma(data.typedData(), data.size())};
  data -= mean;
}

bool write(const GenericRecord &trace, std::ostream &out) {
  IO::MSeedRecord rec{trace};
  int recLength = rec.data()->size() * rec.data()->elementSize() + 64;
  recLength = nextPowerOfTwo<int>(recLength, 128,
                                  1048576);  // MINRECLEN 128, MAXRECLEN 1048576
  if (recLength <= 0) return false;

  try {
    rec.setOutputRecordLength(recLength);
    rec.write(out);
  } catch (std::exception &e) {
    SCDETECT_LOG_WARNING("Failed writing waveform: %s", e.what());
    return false;
  }
  return true;
}

bool read(GenericRecord &trace, std::istream &in) {
  IO::MSeedRecord rec(Array::DOUBLE, Record::Hint::DATA_ONLY);
  try {
    rec.read(in);

    trace = GenericRecord(rec);
    trace.setData(rec.data()->clone());
  } catch (std::exception &e) {
    SCDETECT_LOG_WARNING("Failed reading waveform: %s", e.what());
    return false;
  }
  return true;
}

}  // namespace waveform

WaveformHandlerIface::BaseException::BaseException()
    : Exception{"base waveform handler exception"} {}

const double WaveformHandler::_downloadMargin{2};

void WaveformHandlerIface::process(const GenericRecordPtr &trace,
                                   const ProcessingConfig &config,
                                   const Core::TimeWindow &twTrim) const {
  if (config.demean) {
    waveform::demean(*trace);
  }

  if (config.targetFrequency) {
    waveform::resample(*trace, config.targetFrequency);
  }

  if (!config.filterId.empty()) {
    if (!waveform::filter(*trace, config.filterId)) {
      throw BaseException{Core::stringify(
          "%s: Filtering failed with filter: filter=%s,"
          "start=%s, end=%s",
          trace->streamID().c_str(), config.filterId.c_str(),
          trace->startTime().iso().c_str(), trace->endTime().iso().c_str())};
    }
  }

  if (twTrim) {
    if (!waveform::trim(*trace, twTrim)) {
      throw BaseException{Core::stringify(
          "%s: Incomplete trace; not enough data for requested time:"
          "start=%s, end=%s",
          trace->streamID().c_str(), twTrim.startTime().iso().c_str(),
          twTrim.endTime().iso().c_str())};
    }
  }
}

WaveformHandler::NoData::NoData() : BaseException{"no data avaiable"} {}

WaveformHandler::WaveformHandler(const std::string &recordStreamUrl)
    : _recordStreamUrl(recordStreamUrl) {}

GenericRecordCPtr WaveformHandler::get(const DataModel::WaveformStreamID &id,
                                       const Core::TimeWindow &tw,
                                       const ProcessingConfig &config) {
  return get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr WaveformHandler::get(const DataModel::WaveformStreamID &id,
                                       const Core::Time &start,
                                       const Core::Time &end,
                                       const ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr WaveformHandler::get(const std::string &netCode,
                                       const std::string &staCode,
                                       const std::string &locCode,
                                       const std::string &chaCode,
                                       const Core::Time &start,
                                       const Core::Time &end,
                                       const ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return get(netCode, staCode, locCode, chaCode, tw, config);
}

GenericRecordCPtr WaveformHandler::get(const std::string &netCode,
                                       const std::string &staCode,
                                       const std::string &locCode,
                                       const std::string &chaCode,
                                       const Core::TimeWindow &tw,
                                       const ProcessingConfig &config) {
  try {
    util::WaveformStreamID wfStreamId{netCode, staCode, locCode, chaCode};
  } catch (ValueException &e) {
    throw BaseException{std::string{"invalid waveform stream identifier: "} +
                        e.what()};
  }

  IO::RecordStreamPtr rs = IO::RecordStream::Open(_recordStreamUrl.c_str());
  if (!rs) {
    throw BaseException{
        std::string{"Failed to open RecordStream: " + _recordStreamUrl}};
  }

  Core::TimeSpan downloadMargin{_downloadMargin};
  Core::TimeWindow twWithMargin{tw.startTime() - downloadMargin,
                                tw.endTime() + downloadMargin};
  if (!config.filterId.empty()) {
    Core::TimeSpan margin{config.filterMarginTime};
    twWithMargin.setStartTime(twWithMargin.startTime() - margin);
    twWithMargin.setEndTime(twWithMargin.endTime() + margin);
  }

  rs->setTimeWindow(twWithMargin);
  rs->addStream(netCode, staCode, locCode, chaCode);

  IO::RecordInput inp{rs.get(), Array::DOUBLE, Record::DATA_ONLY};
  std::unique_ptr<RecordSequence> seq{
      util::make_unique<TimeWindowBuffer>(twWithMargin)};
  RecordPtr rec;
  while ((rec = inp.next())) {
    seq->feed(rec.get());
  }
  rs->close();

  if (seq->empty()) {
    throw NoData{Core::stringify(
        "%s.%s.%s.%s: No data: start=%s, end=%s", netCode.c_str(),
        staCode.c_str(), locCode.c_str(), chaCode.c_str(),
        tw.startTime().iso().c_str(), tw.endTime().iso().c_str())};
  }

  GenericRecordPtr trace{seq->contiguousRecord<double>()};
  if (!trace) {
    throw BaseException{Core::stringify(
        "%s.%s.%s.%s: Failed to merge records into single trace: start=%s, "
        "end=%s",
        netCode.c_str(), staCode.c_str(), locCode.c_str(), chaCode.c_str(),
        tw.startTime().iso().c_str(), tw.endTime().iso().c_str())};
  }

  process(trace, config, tw);
  return trace;
}

const std::string Cached::_cacheKeySep{"."};

Cached::Cached(WaveformHandlerIfacePtr waveformHandler, bool raw)
    : _waveformHandler(waveformHandler), _raw(raw) {}

GenericRecordCPtr Cached::get(
    const DataModel::WaveformStreamID &id, const Core::TimeWindow &tw,
    const WaveformHandlerIface::ProcessingConfig &config) {
  return get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr Cached::get(
    const DataModel::WaveformStreamID &id, const Core::Time &start,
    const Core::Time &end,
    const WaveformHandlerIface::ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr Cached::get(
    const std::string &netCode, const std::string &staCode,
    const std::string &locCode, const std::string &chaCode,
    const Core::Time &start, const Core::Time &end,
    const WaveformHandlerIface::ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return get(netCode, staCode, locCode, chaCode, tw, config);
}

GenericRecordCPtr Cached::get(
    const std::string &netCode, const std::string &staCode,
    const std::string &locCode, const std::string &chaCode,
    const Core::TimeWindow &tw,
    const WaveformHandlerIface::ProcessingConfig &config) {
  auto setCache = [&](const std::string &cacheKey,
                      GenericRecordCPtr trace) -> bool {
    if (!set(cacheKey, trace)) {
      SCDETECT_LOG_DEBUG("Failed to cache trace for key: %s", cacheKey.c_str());
      return false;
    }
    return true;
  };

  try {
    util::WaveformStreamID wfStreamId{netCode, staCode, locCode, chaCode};
  } catch (ValueException &e) {
    throw BaseException{std::string{"invalid waveform stream identifier: "} +
                        e.what()};
  }

  std::string cache_key;
  makeCacheKey(netCode, staCode, locCode, chaCode, tw, config, cache_key);

  bool cached = true;
  GenericRecordCPtr trace{get(cache_key)};
  if (!trace) {
    cached = false;

    ProcessingConfig disabled{config};
    disabled.filterId = "";
    disabled.targetFrequency = 0;
    disabled.demean = false;

    Core::TimeWindow corrected{tw};
    if (!config.filterId.empty()) {
      const Core::TimeSpan margin{config.filterMarginTime};
      corrected.setStartTime(tw.startTime() - margin);
      corrected.setEndTime(tw.endTime() + margin);
    }
    trace = _waveformHandler->get(netCode, staCode, locCode, chaCode, corrected,
                                  disabled);
  }

  // cache the raw data
  if (!cached && !cacheProcessed()) {
    setCache(cache_key, trace);
    // TODO (damb): Find a better solution! -> Ideally,
    // `WaveformHandlerIface::Get()` would return a pointer of type
    // `GenericRecordPtr` i.e. a non-const pointer.

    // make sure we do not modified the data cached i.e. create a copy
    trace = util::make_smart<const GenericRecord>(*trace);
  }

  process(const_cast<GenericRecord *>(trace.get()), config, tw);

  // cache processed data
  if (!cached && cacheProcessed()) {
    setCache(cache_key, trace);
  }

  return trace;
}

void Cached::makeCacheKey(const std::string &netCode,
                          const std::string &staCode,
                          const std::string &locCode,
                          const std::string &chaCode,
                          const Core::TimeWindow &tw,
                          const WaveformHandlerIface::ProcessingConfig &config,
                          std::string &result) const {
  Core::TimeWindow twWithMargin{tw};
  if (!cacheProcessed()) {
    if (!config.filterId.empty()) {
      Core::TimeSpan margin{config.filterMarginTime};
      twWithMargin.setStartTime(tw.startTime() - margin);
      twWithMargin.setEndTime(tw.endTime() + margin);
    }
  }

  std::vector<std::string> keyComponents{netCode,
                                         staCode,
                                         locCode,
                                         chaCode,
                                         twWithMargin.startTime().iso(),
                                         twWithMargin.endTime().iso()};

  if (cacheProcessed()) {
    keyComponents.push_back(
        std::to_string(std::hash<ProcessingConfig>{}(config)));
  }

  makeCacheKey(keyComponents, result);
}

void Cached::makeCacheKey(std::vector<std::string> keyComponents,
                          std::string &result) const {
  result = boost::algorithm::join(keyComponents, _cacheKeySep);
}

FileSystemCache::FileSystemCache(WaveformHandlerIfacePtr waveform_handler,
                                 const std::string &path, bool raw)
    : Cached(waveform_handler, raw), _pathCache(path) {}

GenericRecordCPtr FileSystemCache::get(const std::string &key) {
  std::string fpath{(boost::filesystem::path(_pathCache) / key).string()};
  if (!Util::fileExists(fpath)) return nullptr;

  std::ifstream ifs{fpath};
  auto trace{util::make_smart<GenericRecord>()};
  if (!waveform::read(*trace, ifs)) return nullptr;

  return trace;
}

bool Cached::cacheProcessed() const { return !_raw; }

bool FileSystemCache::set(const std::string &key, GenericRecordCPtr value) {
  if (!value) return false;

  std::string fpath{(boost::filesystem::path(_pathCache) / key).string()};
  std::ofstream ofs(fpath);
  if (!waveform::write(*value, ofs)) {
    SCDETECT_LOG_DEBUG("Failed to set cache for file: %s", fpath.c_str());
    return false;
  }
  return true;
}

bool FileSystemCache::exists(const std::string &key) {
  std::string fpath{(boost::filesystem::path(_pathCache) / key).string()};
  return Util::fileExists(fpath);
}

InMemoryCache::InMemoryCache(WaveformHandlerIfacePtr waveformHandler, bool raw)
    : Cached(waveformHandler, raw) {}

GenericRecordCPtr InMemoryCache::get(const std::string &key) {
  const auto it = _cache.find(key);
  if (_cache.end() == it) return nullptr;
  return it->second;
}

bool InMemoryCache::set(const std::string &key, GenericRecordCPtr value) {
  _cache[key] = value;
  return true;
}

bool InMemoryCache::exists(const std::string &key) {
  return _cache.find(key) != _cache.end();
}

}  // namespace detect
}  // namespace Seiscomp

namespace std {

inline std::size_t
hash<Seiscomp::detect::WaveformHandlerIface::ProcessingConfig>::operator()(
    const Seiscomp::detect::WaveformHandlerIface::ProcessingConfig &c)
    const noexcept {
  std::size_t ret{0};
  boost::hash_combine(ret, std::hash<std::string>{}(c.filterId));
  boost::hash_combine(ret, std::hash<double>{}(c.filterMarginTime));
  boost::hash_combine(ret, std::hash<double>{}(c.targetFrequency));
  boost::hash_combine(ret, std::hash<bool>{}(c.demean));
  return ret;
}

}  // namespace std
