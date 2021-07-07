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
#include "../utils.h"

namespace utf = boost::unit_test;
namespace utf_data = utf::data;
namespace utf_tt = boost::test_tools;

constexpr double test_unit_tolerance{0.000001};

namespace Seiscomp {
namespace detect {
namespace test {
namespace ds {

struct Sample {
  using TimeSeries = std::vector<double>;
  // The template waveform data
  TimeSeries template_data;
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

  os << "template: " << Serialize(sample.template_data) << ", data: {";
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
    {/*template_data=*/{1, 2, 1}, /*data=*/{{1, 1}},
     /*expected=*/{{-0.5, 0.5}}},
    {/*template_data=*/{1, 2, 1}, /*data=*/{{1, 1, 1}},
     /*expected=*/{{-0.5, 0.5, 0}}},
    {/*template_data=*/{1, 2, 1}, /*data=*/{{1, 1, 0, 1, 2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*template_data=*/{1, 2, 1}, /*data=*/{{1, 1, 0}, {1, 2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*template_data=*/{1, 2, 1}, /*data=*/{{1, 1, 0, 1}, {2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*template_data=*/{1, 2, 1}, /*data=*/{{1, 1, 0}, {1}, {2, 1, 0}},
     /*expected=*/{{-0.5, 0.5, 0.5, -1, 0, 1, 0}}},
    {/*template_data=*/{1, 2, 1}, /*data=*/{{0, 0, 0, 2, 4, 2, 0}},
     /*expected=*/{{0, 0, 0, -0.5, 0, 1, 0}}},
    {/*template_data=*/{0, 1, 0}, /*data=*/{{0, -1, 0}},
     /*expected=*/{{0, 0.5, -1}}},

};

BOOST_TEST_DECORATOR(*utf::tolerance(test_unit_tolerance))
BOOST_DATA_TEST_CASE(crosscorrelation, utf_data::make(dataset)) {
  // create dummy record
  auto template_trace{utils::make_smart<GenericRecord>(
      "NET", "STA", "LOC", "CHA", Core::Time::GMT(), 1.0)};
  template_trace->setData(static_cast<int>(sample.template_data.size()),
                          sample.template_data.data(), Array::DOUBLE);

  filter::CrossCorrelation<double> xcorr{template_trace};

  std::vector<ds::Sample::TimeSeries> filtered;
  for (auto data : sample.data) {
    xcorr.Apply(data);
    filtered.push_back(data);
  }

  const auto joined{ds::Join(filtered)};
  BOOST_TEST_REQUIRE(joined.size() == sample.expected.size());
  BOOST_TEST(joined == sample.expected, utf_tt::per_element());
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
