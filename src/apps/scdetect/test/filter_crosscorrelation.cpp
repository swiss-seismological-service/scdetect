#define SEISCOMP_TEST_MODULE test_filter_crosscorrelation
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/unittest/unittests.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/test/data/dataset.hpp>
#include <boost/test/data/test_case.hpp>
#include <string>
#include <vector>

#include "../filter/crosscorrelation.h"
#include "../util/memory.h"

namespace utf = boost::unit_test;
namespace utf_data = utf::data;
namespace utf_tt = boost::test_tools;

constexpr double testUnitTolerance{0.000001};

namespace Seiscomp {
namespace detect {
namespace test {
namespace ds {

struct Sample {
  using TimeSeries = std::vector<double>;
  // The template waveform data
  TimeSeries templateData;
  // The data to be filtered
  std::vector<TimeSeries> data;
  // The expected filtered time series
  TimeSeries expected;

  friend std::ostream &operator<<(std::ostream &os, const Sample &sample);
};

std::ostream &operator<<(std::ostream &os, const Sample &sample) {
  const auto Serialize = [](const Sample::TimeSeries &s) {
    return "{" +
           boost::algorithm::join(
               s | boost::adaptors::transformed(
                       [](double d) { return std::to_string(d); }),
               ", ") +
           "}";
  };

  os << "template: " << Serialize(sample.templateData) << ", data: {";
  for (const auto &data : sample.data) {
    os << Serialize(data);
  }
  return os << "}";
}

// Join a nested container of type `R1<R2<T>>` and return a joined container
// `R1<T>`.
template <template <class, class...> class R1,
          template <class, class...> class R2, class T, class... A1,
          class... A2>
R1<T, A2...> Join(R1<R2<T, A2...>, A1...> const &outer) {
  R1<T, A2...> joined;

  joined.reserve(std::accumulate(outer.begin(), outer.end(), std::size_t{},
                                 [](std::size_t size, const R2<T> &inner) {
                                   return size + inner.size();
                                 }));

  for (auto const &inner : outer)
    joined.insert(joined.end(), inner.begin(), inner.end());
  return joined;
}

}  // namespace ds

using Samples = std::vector<ds::Sample>;
Samples dataset{
    {/*templateData=*/{1, 2, 1}, /*data=*/{{1, 1}},
     /*expected=*/{{-0.5, 0.5}}},
    {/*templateData=*/{1, 2, 1}, /*data=*/{{1, 1, 1}},
     /*expected=*/{{-0.5, 0.5, 0}}},
    {/*templateData=*/{1, 2, 1}, /*data=*/{{1, 1, 0, 1, 2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*templateData=*/{1, 2, 1}, /*data=*/{{1, 1, 0}, {1, 2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*templateData=*/{1, 2, 1}, /*data=*/{{1, 1, 0, 1}, {2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*templateData=*/{1, 2, 1}, /*data=*/{{1, 1, 0}, {1}, {2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*templateData=*/{1, 2, 1}, /*data=*/{{0, 0, 0, 2, 4, 2, 0}},
     /*expected=*/{{0, 0, 0, -0.5, 0, 1, 0}}},
    {/*templateData=*/{0, 1, 0}, /*data=*/{{0, -1, 0}},
     /*expected=*/{{0, 0.5, -1}}},

};

BOOST_TEST_DECORATOR(*utf::tolerance(testUnitTolerance))
BOOST_DATA_TEST_CASE(crosscorrelation, utf_data::make(dataset)) {
  // create dummy record
  auto templateTrace{util::make_smart<GenericRecord>("NET", "STA", "LOC", "CHA",
                                                     Core::Time::GMT(), 1.0)};
  templateTrace->setData(static_cast<int>(sample.templateData.size()),
                         sample.templateData.data(), Array::DOUBLE);

  filter::CrossCorrelation<double> xcorr{templateTrace};

  std::vector<ds::Sample::TimeSeries> filtered;
  for (auto data : sample.data) {
    xcorr.apply(data);
    filtered.push_back(data);
  }

  const auto joined{ds::Join(filtered)};
  BOOST_TEST_REQUIRE(joined.size() == sample.expected.size());
  BOOST_TEST(joined == sample.expected, utf_tt::per_element());
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
