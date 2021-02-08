#include "waveform.h"

#include <fstream>
#include <memory>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>

#include <seiscomp/core/strings.h>
#include <seiscomp/io/recordinput.h>
#include <seiscomp/io/records/mseedrecord.h>
#include <seiscomp/io/recordstream.h>
#include <seiscomp/math/filter.h>
#include <seiscomp/utils/files.h>

#include "log.h"
#include "seiscomp/core/datetime.h"
#include "seiscomp/core/timewindow.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {
namespace waveform {

namespace {

template <class T> T NextPowerOfTwo(T a, T min = 1, T max = 1 << 31) {
  int b = min;
  while (b < a) {
    b <<= 1;
    if (b > max)
      return -1;
  }
  return b;
}

} // namespace

bool Merge(GenericRecord &trace, const RecordSequence &seq) {
  RecordCPtr first = seq.front();
  RecordCPtr last;
  double sampling_freq = first->samplingFrequency();
  Core::TimeSpan max_allowed_gap{static_cast<double>(0.5 / sampling_freq)};
  Core::TimeSpan max_allowed_overlap{static_cast<double>(-0.5 / sampling_freq)};

  trace.setNetworkCode(first->networkCode());
  trace.setStationCode(first->stationCode());
  trace.setLocationCode(first->locationCode());
  trace.setChannelCode(first->channelCode());

  trace.setStartTime(first->startTime());
  trace.setSamplingFrequency(sampling_freq);

  Array::DataType datatype{first->data()->dataType()};
  ArrayPtr arr{ArrayFactory::Create(datatype, datatype, 0, nullptr)};

  std::string stream_id{trace.streamID()};

  for (const RecordCPtr &rec : seq) {
    if (rec->samplingFrequency() != sampling_freq) {
      SCDETECT_LOG_WARNING(
          "%s: Inconsistent record sampling frequencies: %f != %f",
          std::string{stream_id}.c_str(), sampling_freq,
          rec->samplingFrequency());
      return false;
    }

    // Check for gaps and overlaps
    if (last) {
      Core::TimeSpan diff{rec->startTime() - last->endTime()};
      if (diff > max_allowed_gap) {
        SCDETECT_LOG_WARNING("%s: Gap detected: %d.%06ds",
                             std::string{stream_id}.c_str(),
                             static_cast<int>(diff.seconds()),
                             static_cast<int>(diff.microseconds()));
        return false;
      }

      if (diff < max_allowed_overlap) {
        SCDETECT_LOG_WARNING("%s: Overlap detected: %fs",
                             std::string{stream_id}.c_str(),
                             static_cast<double>(diff));
        return false;
      }
    }

    arr->append((Array *)(rec->data()));
    last = rec;
  }

  trace.setData(arr.get());
  return true;
}

bool Trim(GenericRecord &trace, const Core::TimeWindow &tw) {
  auto offset{
      static_cast<int>(static_cast<double>(tw.startTime() - trace.startTime()) *
                       trace.samplingFrequency())};
  auto samples{static_cast<int>(tw.length() * trace.samplingFrequency())};

  // Not enough data at start of time window
  if (offset < 0) {
    SCDETECT_LOG_WARNING("%s: Need %d more samples in past.",
                         trace.streamID().c_str(), -offset);
    return false;
  }

  // Not enough data at end of time window
  if (offset + samples > trace.data()->size()) {
    SCDETECT_LOG_WARNING("%s: Need %d more samples past the end.",
                         trace.streamID().c_str(),
                         -(trace.data()->size() - samples - offset));
    return false;
  }

  trace.setStartTime(tw.startTime());
  trace.setData(trace.data()->slice(offset, offset + samples));

  return true;
}

bool Filter(GenericRecord &trace, const std::string &filter_string) {
  if (filter_string.empty())
    return false;

  DoubleArrayPtr data{DoubleArray::Cast(trace.data())};

  std::string filter_error;
  auto filter = Math::Filtering::InPlaceFilter<double>::Create(filter_string,
                                                               &filter_error);
  if (!filter) {
    SCDETECT_LOG_WARNING("Filter creation failed for '%s': %s",
                         filter_string.c_str(), filter_error.c_str());
    return false;
  }
  filter->setSamplingFrequency(trace.samplingFrequency());
  filter->apply(data->size(), data->typedData());
  delete filter;
  trace.dataUpdated();

  return true;
}

void Resample(GenericRecord &trace, double sampling_frequency) {
  if (sampling_frequency <= 0 ||
      trace.samplingFrequency() == sampling_frequency)
    return;

  DoubleArrayPtr data{DoubleArray::Cast(trace.data())};
  Resample(data, trace.samplingFrequency(), sampling_frequency);

  trace.setSamplingFrequency(static_cast<double>(sampling_frequency));
  trace.dataUpdated();
}

void Resample(DoubleArrayPtr dataArr, double sampling_frequency_from,
              double sampling_frequency_to) {
  const double resamp_factor = sampling_frequency_to / sampling_frequency_from;
  const double nyquist =
      std::min(sampling_frequency_to, sampling_frequency_from) / 2.;

  const double *data = dataArr->typedData();
  const int data_size = dataArr->size();

  const int resampled_data_size = data_size * resamp_factor;
  double resampled_data[resampled_data_size];

  /*
   * Compute one sample of the resampled data:
   *
   * x: new sample point location (relative to old indexes)
   *    (e.g. every other integer for 0.5x decimation)
   * fmax: low pass filter cutoff frequency. Fmax should be less
   *       than half of data_freq, and less than half of the
   *       new sample frequency (the reciprocal of the x step size).
   * win_len: width of windowed Sinc used as the low pass filter
   *          Filter quality increases with a larger window width.
   *          The wider the window, the closer fmax can approach half of
   *          data_freq or the new sample frequency
   */
  auto new_sample = [&data, &data_size, &sampling_frequency_from](
                        double x, double fmax, double win_len) -> double {
    double r_g =
        2 * fmax / sampling_frequency_from;  // Calc gain correction factor
    double r_y = 0;

    // For 1 window width
    for (double win_i = -(win_len / 2.); win_i < (win_len / 2.); win_i += 1.) {
      int j = int(x + win_i);  // input sample index
      if (j >= 0 && j < data_size) {
        // calculate von Hann Window. Scale and calculate Sinc
        double r_w = 0.5 - 0.5 * std::cos(2 * M_PI * (0.5 + (j - x) / win_len));
        double r_a = 2 * M_PI * (j - x) * fmax / sampling_frequency_from;
        double r_snc = (r_a != 0) ? std::sin(r_a) / r_a : 1;
        r_y = r_y + r_g * r_w * r_snc * data[j];
      }
    }
    return r_y;
  };

  for (int i = 0; i < resampled_data_size; i++) {
    /*
     * If the x step size is rational the same Window and Sinc values
     * will be recalculated repeatedly. Therefore these values can either
     * be cached, or pre-calculated and stored in a table (polyphase
     * interpolation); or interpolated from a smaller pre-calculated table;
     * or computed from a set of low-order polynomials fitted to each
     * section or lobe between  zero-crossings of the windowed Sinc (Farrow)
     */
    double x = i / resamp_factor;
    resampled_data[i] = new_sample(x, nyquist, 4);
  }

  dataArr->setData(resampled_data_size, resampled_data);
}

void Demean(GenericRecord &trace) {
  DoubleArrayPtr data{DoubleArray::Cast(trace.data())};
  Demean(data);
  trace.dataUpdated();
}

void Demean(DoubleArrayPtr data) { *data -= data->mean(); }

bool Write(const GenericRecord &trace, std::ostream &out) {
  IO::MSeedRecord rec{trace};
  int rec_length = rec.data()->size() * rec.data()->elementSize() + 64;
  rec_length = NextPowerOfTwo<int>(rec_length, 128,
                                   1048576); // MINRECLEN 128, MAXRECLEN 1048576
  if (rec_length <= 0)
    return false;

  try {
    rec.setOutputRecordLength(rec_length);
    rec.write(out);
  } catch (std::exception &e) {
    SCDETECT_LOG_WARNING("Failed writing waveform: %s", e.what());
    return false;
  }
  return true;
}

bool Read(GenericRecord &trace, std::istream &in) {
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

} // namespace waveform

WaveformHandlerIface::BaseException::BaseException()
    : Exception{"base waveform handler exception"} {}

WaveformHandler::NoData::NoData() : BaseException{"no data avaiable"} {}

WaveformHandler::WaveformHandler(const std::string &record_stream_url)
    : record_stream_url_(record_stream_url) {}

GenericRecordCPtr WaveformHandler::Get(const DataModel::WaveformStreamID &id,
                                       const Core::TimeWindow &tw,
                                       const ProcessingConfig &config) {
  return Get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr WaveformHandler::Get(const DataModel::WaveformStreamID &id,
                                       const Core::Time &start,
                                       const Core::Time &end,
                                       const ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return Get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr
WaveformHandler::Get(const std::string &net_code, const std::string &sta_code,
                     const std::string &loc_code, const std::string &cha_code,
                     const Core::Time &start, const Core::Time &end,
                     const ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return Get(net_code, sta_code, loc_code, cha_code, tw, config);
}

GenericRecordCPtr WaveformHandler::Get(const std::string &net_code,
                                       const std::string &sta_code,
                                       const std::string &loc_code,
                                       const std::string &cha_code,
                                       const Core::TimeWindow &tw,
                                       const ProcessingConfig &config) {

  utils::WaveformStreamID wf_stream_id{net_code, sta_code, loc_code, cha_code};
  if (!wf_stream_id.IsValid()) {
    throw BaseException{"Invalid waveform stream identifier."};
  }

  IO::RecordStreamPtr rs = IO::RecordStream::Open(record_stream_url_.c_str());
  if (!rs) {
    throw BaseException{
        std::string{"Failed to open RecordStream: " + record_stream_url_}};
  }

  Core::TimeSpan download_margin{download_margin_};
  Core::TimeWindow tw_with_margin{tw.startTime() - download_margin,
                                  tw.endTime() + download_margin};

  rs->setTimeWindow(tw_with_margin);
  rs->addStream(net_code, sta_code, loc_code, cha_code);

  IO::RecordInput inp{rs.get(), Array::DOUBLE, Record::DATA_ONLY};
  std::unique_ptr<RecordSequence> seq{utils::make_unique<TimeWindowBuffer>(tw)};
  RecordPtr rec;
  while (rec = inp.next()) {
    seq->feed(rec.get());
  }
  rs->close();

  if (seq->empty()) {
    throw NoData{Core::stringify(
        "%s.%s.%s.%s: No data: start=%s, end=%s", net_code.c_str(),
        sta_code.c_str(), loc_code.c_str(), cha_code.c_str(),
        tw.startTime().iso().c_str(), tw.endTime().iso().c_str())};
  }

  // merge RecordSequence into GenericRecord
  auto trace{utils::make_smart<GenericRecord>()};
  if (!waveform::Merge(*trace, *seq)) {
    throw BaseException{Core::stringify(
        "%s.%s.%s.%s: Failed to merge records into single trace: start=%s, "
        "end=%s",
        net_code.c_str(), sta_code.c_str(), loc_code.c_str(), cha_code.c_str(),
        tw.startTime().iso().c_str(), tw.endTime().iso().c_str())};
  }

  trace->setChannelCode(cha_code);
  if (!waveform::Trim(*trace, tw)) {
    throw BaseException{Core::stringify(
        "%s.%s.%s.%s: Incomplete trace; not enough data for requested time:"
        "start=%s, end=%s",
        net_code.c_str(), sta_code.c_str(), loc_code.c_str(), cha_code.c_str(),
        tw.startTime().iso().c_str(), tw.endTime().iso().c_str())};
  }

  return trace;
}

Cached::Cached(WaveformHandlerIfacePtr waveform_handler, bool raw)
    : waveform_handler_(waveform_handler), raw_(raw) {}

GenericRecordCPtr
Cached::Get(const DataModel::WaveformStreamID &id, const Core::TimeWindow &tw,
            const WaveformHandlerIface::ProcessingConfig &config) {
  return Get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr
Cached::Get(const DataModel::WaveformStreamID &id, const Core::Time &start,
            const Core::Time &end,
            const WaveformHandlerIface::ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return Get(id.networkCode(), id.stationCode(), id.locationCode(),
             id.channelCode(), tw, config);
}

GenericRecordCPtr
Cached::Get(const std::string &net_code, const std::string &sta_code,
            const std::string &loc_code, const std::string &cha_code,
            const Core::Time &start, const Core::Time &end,
            const WaveformHandlerIface::ProcessingConfig &config) {
  Core::TimeWindow tw{start, end};
  return Get(net_code, sta_code, loc_code, cha_code, tw, config);
}

GenericRecordCPtr
Cached::Get(const std::string &net_code, const std::string &sta_code,
            const std::string &loc_code, const std::string &cha_code,
            const Core::TimeWindow &tw,
            const WaveformHandlerIface::ProcessingConfig &config) {

  auto SetCache = [&](const std::string &cache_key,
                      GenericRecordCPtr trace) -> bool {
    if (!Set(cache_key, trace)) {
      SCDETECT_LOG_DEBUG("Failed to cache trace for key: %s",
                         cache_key.c_str());
      return false;
    }
    return true;
  };

  utils::WaveformStreamID wf_stream_id{net_code, sta_code, loc_code, cha_code};
  if (!wf_stream_id.IsValid()) {
    throw BaseException{"Invalid waveform stream identifier."};
  }

  std::string cache_key;
  MakeCacheKey(net_code, sta_code, loc_code, cha_code, tw, config, cache_key);

  bool cached = true;
  GenericRecordCPtr trace{Get(cache_key)};
  if (!trace) {
    cached = false;
    trace = waveform_handler_->Get(net_code, sta_code, loc_code, cha_code, tw,
                                   config);
  }

  if (!cached && !CacheProcessed()) {
    SetCache(cache_key, trace);
  }

  auto trace_ptr = const_cast<GenericRecord *>(trace.get());
  if (config.demean)
    waveform::Demean(*trace_ptr);

  if (config.resample_frequency)
    waveform::Resample(*trace_ptr, config.resample_frequency);

  if (!config.filter_string.empty()) {
    if (!waveform::Filter(*trace_ptr, config.filter_string)) {
      throw BaseException{Core::stringify(
          "%s.%s.%s.%s: Filtering failed with filter: filter=%s,"
          "start=%s, end=%s",
          net_code.c_str(), sta_code.c_str(), loc_code.c_str(),
          cha_code.c_str(), config.filter_string.c_str())};
    }
  }

  if (!cached && CacheProcessed()) {
    SetCache(cache_key, trace);
  }

  return trace;
}

void Cached::MakeCacheKey(const std::string &net_code,
                          const std::string &sta_code,
                          const std::string &loc_code,
                          const std::string &cha_code,
                          const Core::TimeWindow &tw,
                          const WaveformHandlerIface::ProcessingConfig &config,
                          std::string &result) {
  auto BoolToString = [](const bool b) -> std::string {
    std::stringstream ss;
    ss << std::boolalpha << b;
    return ss.str();
  };

  std::vector<std::string> key_components{
      net_code,          sta_code, loc_code, cha_code, tw.startTime().iso(),
      tw.endTime().iso()};

  if (CacheProcessed()) {
    key_components.push_back(config.filter_string);
    key_components.push_back(std::to_string(config.resample_frequency));
    key_components.push_back(BoolToString(config.demean));
  }

  MakeCacheKey(key_components, result);
}

void Cached::MakeCacheKey(std::vector<std::string> key_components,
                          std::string &result) {
  result = boost::algorithm::join(key_components, cache_key_sep_);
}

FileSystemCache::FileSystemCache(WaveformHandlerIfacePtr waveform_handler,
                                 const std::string &path, bool raw)
    : Cached(waveform_handler, raw), path_cache_(path) {}

GenericRecordCPtr FileSystemCache::Get(const std::string &key) {
  std::string fpath{(boost::filesystem::path(path_cache_) / key).string()};
  if (!Util::fileExists(fpath))
    return nullptr;

  std::ifstream ifs{fpath};
  auto trace{utils::make_smart<GenericRecord>()};
  if (!waveform::Read(*trace, ifs))
    return nullptr;

  return trace;
}

bool Cached::CacheProcessed() const { return !raw_; }

bool FileSystemCache::Set(const std::string &key, GenericRecordCPtr value) {
  if (!value)
    return false;

  std::string fpath{(boost::filesystem::path(path_cache_) / key).string()};
  std::ofstream ofs(fpath);
  if (!waveform::Write(*value, ofs)) {
    SCDETECT_LOG_DEBUG("Failed to set cache for file: %s", fpath.c_str());
    return false;
  }
  return true;
}

bool FileSystemCache::Exists(const std::string &key) {
  std::string fpath{(boost::filesystem::path(path_cache_) / key).string()};
  return Util::fileExists(fpath);
}

InMemoryCache::InMemoryCache(WaveformHandlerIfacePtr waveform_handler, bool raw)
    : Cached(waveform_handler, raw) {}

GenericRecordCPtr InMemoryCache::Get(const std::string &key) {
  const auto it = cache_.find(key);
  if (cache_.end() == it)
    return nullptr;
  return it->second;
}

bool InMemoryCache::Set(const std::string &key, GenericRecordCPtr value) {
  cache_[key] = value;
  return true;
}

bool InMemoryCache::Exists(const std::string &key) {
  return cache_.find(key) != cache_.end();
}

} // namespace detect
} // namespace Seiscomp
