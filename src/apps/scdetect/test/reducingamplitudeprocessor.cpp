#define SEISCOMP_TEST_MODULE test_reducingamplitudeprocessor
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>
#include <seiscomp/io/recordstream/file.h>
#include <seiscomp/processing/stream.h>
#include <seiscomp/unittest/unittests.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/optional/optional.hpp>
#include <boost/test/data/dataset.hpp>
#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/tools/interface.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/variant2/variant.hpp>
#include <functional>
#include <initializer_list>
#include <string>
#include <unordered_map>
#include <vector>

#include "../amplitudeprocessor.h"
#include "../utils.h"
#include "../waveformprocessor.h"
#include "fixture.h"
#include "utils.h"

namespace utf = boost::unit_test;
namespace utf_data = utf::data;
namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

namespace detail {

class Buffer : public Seiscomp::RecordSequence {
 public:
  bool feed(const Record *record) {
    // XXX(damb): assume records are already sorted
    push_back(record);
    return true;
  }
  RecordSequence *copy() const { return nullptr; }
  RecordSequence *clone() const { return nullptr; }
};

}  // namespace detail

namespace ds {

struct Sample {
  using WaveformBuffers = std::unordered_map<std::string, detail::Buffer>;
  using WaveformLoader = std::function<void(WaveformBuffers &)>;
  using WaveformDataSource = boost::variant2::variant<fs::path, WaveformLoader>;

  Sample(const std::string &description,
         const WaveformProcessor::PublishResultCallback &validatorCallback,
         const fs::path &pathDataSource,
         WaveformProcessor::Status expectedStatus,
         const boost::optional<Core::TimeWindow> &timeWindow = boost::none)
      : description{description},
        waveformDataSource{pathDataSource},
        validatorCallback{validatorCallback},
        expectedStatus{expectedStatus},
        timeWindow{timeWindow} {}
  Sample(const std::string &description,
         const WaveformProcessor::PublishResultCallback &validatorCallback,
         const WaveformLoader &waveformLoader,
         WaveformProcessor::Status expectedStatus,
         const boost::optional<Core::TimeWindow> &timeWindow = boost::none)
      : description{description},
        waveformDataSource{waveformLoader},
        validatorCallback{validatorCallback},
        expectedStatus{expectedStatus},
        timeWindow{timeWindow} {}

  // The sample's description
  std::string description;

  WaveformDataSource waveformDataSource;
  // Callback evaluating the test
  WaveformProcessor::PublishResultCallback validatorCallback;
  // Defines the processor's eventually expected status
  WaveformProcessor::Status expectedStatus;
  // Defines an optional time window for processing
  boost::optional<Core::TimeWindow> timeWindow;

  // Initializes the sample and loads the waveform data
  //
  // - must be called at the very beginning of the test
  void init(const fs::path &pathData) {
    try {
      waveformDataSource = fs::path{pathData} /
                           boost::variant2::get<fs::path>(waveformDataSource);
    } catch (boost::variant2::bad_variant_access &) {
    }

    boost::variant2::visit(Loader(this), waveformDataSource);
  }

  // Returns a reference to the buffer
  const detail::Buffer &at(
      const utils::WaveformStreamID &waveformStreamId) const {
    return _waveformBuffers.at(utils::to_string(waveformStreamId));
  }

  // Returns the waveform stream identifiers regarding the waveform data
  std::vector<utils::WaveformStreamID> waveformStreamIds() const {
    std::vector<utils::WaveformStreamID> retval;
    for (const auto &_waveformBuffersPair : _waveformBuffers) {
      retval.push_back(utils::WaveformStreamID{_waveformBuffersPair.first});
    }
    return retval;
  }
  // Returns the time window to be used for the test sample
  boost::optional<Core::TimeWindow> signalTimeWindow() const {
    if (timeWindow) {
      return *timeWindow;
    }
    return commonTimeWindow();
  }

  // Returns the common time window regarding all the waveform data
  boost::optional<Core::TimeWindow> commonTimeWindow() const {
    if (_waveformBuffers.empty()) {
      return boost::none;
    }

    using PairType = decltype(_waveformBuffers)::value_type;
    auto itEndTime{
        std::min_element(_waveformBuffers.begin(), _waveformBuffers.end(),
                         [](const PairType &lhs, const PairType &rhs) {
                           return lhs.second.timeWindow().endTime() <
                                  rhs.second.timeWindow().endTime();
                         })};
    auto itStartTime{
        std::max_element(_waveformBuffers.begin(), _waveformBuffers.end(),
                         [](const PairType &lhs, const PairType &rhs) {
                           return lhs.second.timeWindow().startTime() <
                                  rhs.second.timeWindow().startTime();
                         })};
    return Core::TimeWindow{itStartTime->second.timeWindow().startTime(),
                            itEndTime->second.timeWindow().endTime()};
  }

  friend std::ostream &operator<<(std::ostream &os, const Sample &sample) {
    os << "\ndescription: \"" << sample.description << "\"\n";
    if (sample._waveformBuffers.empty()) {
      os << "waveform data:\n  no buffered data available (loaded during "
            "sample initialization)\n";
    } else {
      os << "waveform data:\n";
      for (const auto &bufferPair : sample._waveformBuffers) {
        const auto &tw{bufferPair.second.timeWindow()};
        os << "  " << bufferPair.first << " | " << tw.startTime().iso() << " - "
           << tw.endTime().iso() << " | " << bufferPair.second.gaps().size()
           << " gaps\n";
      }
    }
    return os;
  }

 private:
  // Implements waveform data loading facilities
  class Loader {
   public:
    Loader(Sample *sample) : _sample{sample} {}

    void operator()(const fs::path &pathWaveformData) {
      IO::RecordStreamPtr recordStream{IO::RecordStream::Open(
          std::string{"file://" + pathWaveformData.string()}.c_str())};

      if (!recordStream) {
        BOOST_FAIL("Failed to create recordStream");
      }

      bool success{true};
      RecordPtr record;
      while ((record = recordStream->next())) {
        auto &buffer{_sample->_waveformBuffers[record->streamID()]};
        if (!buffer.feed(record.get())) {
          success = false;
          break;
        }
      }

      if (!success) {
        _sample->_waveformBuffers.clear();
        BOOST_FAIL("Failed to buffer samples from recordStream");
      }
    }

    void operator()(const WaveformLoader &loader) {
      loader(_sample->_waveformBuffers);
    }

   private:
    Sample *_sample;
  };

  friend class Loader;

  WaveformBuffers _waveformBuffers;
};

}  // namespace ds

// Concrete implementation of `ReducingAmplitudeProcessor` for test purposes
//
// - does not implement any preprocessing
class TestReducingAmplitudeProcessor : public ReducingAmplitudeProcessor {
 public:
  TestReducingAmplitudeProcessor(const std::string &id,
                                 const Core::TimeWindow &tw)
      : ReducingAmplitudeProcessor{id} {
    setTimeWindow(tw);
    _type = "Mtest";
    _unit = "M/S";
  }

  void computeTimeWindow() override {
    // XXX(damb): do nothing; use the time window passed during construction,
    // instead
  }

 protected:
  // Compute the amplitude from `data` simply by means of returning the maximum
  void computeAmplitude(const DoubleArray &data, const IndexRange &idxRange,
                        Amplitude &amplitude) override {
    auto rangeBegin{data.begin() + idxRange.begin};
    auto rangeEnd{data.begin() + idxRange.end};

    const auto it{std::max_element(rangeBegin, rangeEnd)};
    if (it == data.end()) {
      setStatus(Status::kError, 0);
      return;
    }
    amplitude.value.value = *it;
  }

  // Reduces `data` by means of simply computing the sum for the samples
  DoubleArrayCPtr reduceAmplitudeData(
      const std::vector<DoubleArray const *> &data,
      const std::vector<NoiseInfo> &noiseInfos,
      const IndexRange &idxRange) override {
    if (data.size() != noiseInfos.size()) {
      return nullptr;
    }

    const auto numberOfStreams{data.size()};

    std::vector<double> samples;
    for (size_t i = idxRange.begin; i <= idxRange.end; ++i) {
      double sum{0};
      for (size_t j = 0; j < numberOfStreams; ++j) {
        sum += data[j]->get(i);
      }
      samples.push_back(sum);
    }

    return utils::make_smart<DoubleArray>(static_cast<int>(samples.size()),
                                          samples.data());
  }
};

// samples for parameterized testing
using Samples = std::vector<ds::Sample>;
Samples dataset{
    {/*description=*/"single stream, single record (constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {
       auto amplitude{
           boost::dynamic_pointer_cast<const AmplitudeProcessor::Amplitude>(
               result)};
       BOOST_TEST_CHECK(amplitude->value.value == 1.0);
     },
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       auto record{makeRecord<Array::INT>(120, 1, Core::Time::GMT(), 1)};
       auto &buffer{buffers[record->streamID()]};
       if (!buffer.feed(record.get())) {
         BOOST_FAIL("Failed to feed record");
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kFinished},
    {/*description=*/
     "single stream, single record (too short, constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {},
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       const auto startTime{
           Core::Time::FromString("2020-01-01T00:00:00", "%FT%T")};
       auto record{makeRecord<Array::INT>(120, 1, startTime, 1)};
       auto &buffer{buffers[record->streamID()]};
       if (!buffer.feed(record.get())) {
         BOOST_FAIL("Failed to feed record");
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kWaitingForData,
     /*timeWindow=*/
     Core::TimeWindow{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T"),
                      180}},
    {/*description=*/
     "single stream, multi records (equal length, constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {
       auto amplitude{
           boost::dynamic_pointer_cast<const AmplitudeProcessor::Amplitude>(
               result)};
       BOOST_TEST_CHECK(amplitude->value.value == 2.0);
     },
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       size_t sampleCount{60};
       double samplingFrequency{1};
       Core::TimeSpan recordDuration{sampleCount / samplingFrequency};
       auto startTime{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T")};
       for (int i = 0; i < 3; ++i) {
         auto record{makeRecord<Array::INT>(sampleCount, 2, startTime,
                                            samplingFrequency)};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
         startTime += recordDuration;
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kFinished},
    {/*description=*/"single stream, multi records (not equal length, constant "
                     "sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {
       auto amplitude{
           boost::dynamic_pointer_cast<const AmplitudeProcessor::Amplitude>(
               result)};
       BOOST_TEST_CHECK(amplitude->value.value == 2.0);
     },
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       auto startTime{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T")};
       double samplingFrequency{1};
       for (int i = 0; i < 3; ++i) {
         auto sampleCount{static_cast<size_t>(60 * (i + 1))};
         auto record{makeRecord<Array::INT>(sampleCount, 2, startTime,
                                            samplingFrequency)};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }

         Core::TimeSpan recordDuration{sampleCount / samplingFrequency};
         startTime += recordDuration;
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kFinished,
     /*timeWindow=*/
     Core::TimeWindow{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T"),
                      360}},
    {/*description=*/"multi streams, single record (constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {
       auto amplitude{
           boost::dynamic_pointer_cast<const AmplitudeProcessor::Amplitude>(
               result)};
       BOOST_TEST_CHECK(amplitude->value.value == 15.0);
     },
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       const auto now{Core::Time::GMT()};
       // load three streams (each with a single record)
       for (int i = 0; i < 3; ++i) {
         auto record{makeRecord<Array::INT>(
             120, 5, now, /*samplingFrequency=*/1, "C" + std::to_string(i))};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kFinished},
    {/*description=*/"multi streams, multi records (equal length, constant "
                     "sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {
       auto amplitude{
           boost::dynamic_pointer_cast<const AmplitudeProcessor::Amplitude>(
               result)};
       BOOST_TEST_CHECK(amplitude->value.value == 15.0);
     },
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       size_t countSamples{60};
       double samplingFrequency{1};
       const Core::TimeSpan recordDuration{countSamples / samplingFrequency};
       auto startTime{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T")};
       // load three streams (each with three records)
       for (int j = 0; j < 3; ++j) {
         for (int i = 0; i < 3; ++i) {
           auto record{makeRecord<Array::INT>(countSamples, 5, startTime,
                                              samplingFrequency,
                                              "C" + std::to_string(i))};
           auto &buffer{buffers[record->streamID()]};
           if (!buffer.feed(record.get())) {
             BOOST_FAIL("Failed to feed record");
           }
         }
         startTime += recordDuration;
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kFinished,
     /*timeWindow=*/
     Core::TimeWindow{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T"),
                      180}},
    {/*description=*/"multi streams, single record (one of them is too short, "
                     "constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {},
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       double samplingFrequency{1};
       auto startTime{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T")};
       // load streams
       {
         auto record{makeRecord<Array::INT>(180, 5, startTime,
                                            samplingFrequency, "C0")};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
       {
         auto record{makeRecord<Array::INT>(179, 5, startTime,
                                            samplingFrequency, "C1")};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kWaitingForData,
     /*timeWindow=*/
     Core::TimeWindow{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T"),
                      180}},
    {/*description=*/"multi streams, single record (different sampling "
                     "frequency, constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {},
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       auto startTime{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T")};
       // load streams
       {
         auto record{makeRecord<Array::INT>(120, 1, startTime, 10, "C0")};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
       {
         auto record{makeRecord<Array::INT>(120, 2, startTime, 20, "C1")};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kInvalidSamplingFreq,
     /*timeWindow=*/
     Core::TimeWindow{Core::Time::FromString("2020-01-01T00:00:00", "%FT%T"),
                      120}},
    {/*description=*/"multi streams, single record (different record start "
                     "time, constant sample values)",
     /*validatorCallback=*/
     [](const WaveformProcessor *proc, const Record *record,
        const WaveformProcessor::ResultCPtr &result) {
       auto amplitude{
           boost::dynamic_pointer_cast<const AmplitudeProcessor::Amplitude>(
               result)};
       BOOST_TEST_CHECK(amplitude->value.value == 3.0);
       BOOST_TEST_CHECK(
           amplitude->time.reference.iso() ==
           Core::Time::FromString("2020-01-01T00:01:00", "%FT%T").iso());
       BOOST_TEST_CHECK(amplitude->time.begin == 0.0);
       BOOST_TEST_CHECK(amplitude->time.end == 120.0);
     },
     /*waveformLoader=*/
     [](ds::Sample::WaveformBuffers &buffers) {
       double samplingFrequency{1};
       // load streams
       {
         auto startTime{Core::Time::FromString("2020-01-01T00:01:00", "%FT%T")};
         auto record{makeRecord<Array::INT>(120, 1, startTime,
                                            samplingFrequency, "C0")};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
       {
         auto startTime{Core::Time::FromString("2020-01-01T00:00:59", "%FT%T")};
         auto record{makeRecord<Array::INT>(121, 2, startTime,
                                            samplingFrequency, "C1")};
         auto &buffer{buffers[record->streamID()]};
         if (!buffer.feed(record.get())) {
           BOOST_FAIL("Failed to feed record");
         }
       }
     },
     /*expectedStatus=*/WaveformProcessor::Status::kFinished,
     /*timeWindow=*/
     Core::TimeWindow{Core::Time::FromString("2020-01-01T00:01:00", "%FT%T"),
                      120}},
};

BOOST_TEST_GLOBAL_FIXTURE(CLIParserFixture);

BOOST_DATA_TEST_CASE(reducingamplitudeprocessor, utf_data::make(dataset)) {
  // XXX(damb): in the body of the test case, the samples of the dataset are
  // taken by the variable `sample`.
  // XXX(damb): const_cast is required in order to finaliza the sample
  // initialization procedure
  // TODO(damb): Make use of a lazily (delayed) generated dataset, instead:
  // https://www.boost.org/doc/libs/1_77_0/libs/test/doc/html/boost_test/
  // tests_organization/test_cases/test_case_generation/datasets.html
  // #boost_test.tests_organization.test_cases.test_case_generation.datasets.
  // dataset_creation_and_delayed_cre
  const_cast<ds::Sample &>(sample).init(CLIParserFixture::pathData);

  if (!sample.signalTimeWindow()) {
    BOOST_FAIL("Missing signal time window.");
  }

  TestReducingAmplitudeProcessor proc{utils::createUUID(),
                                      *sample.signalTimeWindow()};
  const auto &waveformStreamIds{sample.waveformStreamIds()};
  // initialize the processor
  for (const auto &waveformStreamId : waveformStreamIds) {
    Processing::Stream stream;
    stream.setCode(waveformStreamId.chaCode());
    proc.add(waveformStreamId.netCode(), waveformStreamId.staCode(),
             waveformStreamId.locCode(), stream,
             AmplitudeProcessor::DeconvolutionConfig{});
  }
  proc.setResultCallback(sample.validatorCallback);

  // feed data
  for (const auto &waveformstreamId : waveformStreamIds) {
    auto buffer{sample.at(waveformstreamId)};
    auto it{buffer.begin()};
    while (it != buffer.end()) {
      proc.feed(it->get());
      ++it;
    }
  }

  BOOST_CHECK(proc.status() == sample.expectedStatus);
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
