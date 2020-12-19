#define SEISCOMP_TEST_MODULE test_template_xcorr

#include <array>
#include <numeric>

#include <seiscomp/unittest/unittests.h>

#include "../template.h"
#include "../utils.h"

namespace utf = boost::unit_test;

constexpr double test_unit_tolerance{0.000001};

namespace Seiscomp {
namespace detect {

BOOST_AUTO_TEST_CASE(xcorr, *utf::tolerance(test_unit_tolerance)) {

  auto SumTr1 = [](const std::array<double, 3> &tr1) {
    return std::accumulate(tr1.begin(), tr1.end(), 0.0);
  };
  auto SquaredSumTr1 = [](const std::array<double, 3> &tr1) {
    return std::accumulate(tr1.begin(), tr1.end(), 0.0,
                           [](double a, double b) { return a + b * b; });
  };

  constexpr auto sampling_freq{1.0};

  // template waveform trace
  constexpr auto size_tr1{3};
  std::array<double, size_tr1> tr1{1, 2, 1};
  auto sum_tr1{SumTr1(tr1)};
  auto squared_sum_tr1{SquaredSumTr1(tr1)};

  // real-time trace
  const std::array<double, 5> tr2_01210{0, 1, 2, 1, 0};

  auto xcorr_result{utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{})};

  BOOST_TEST_CHECK(true == template_detail::XCorr(
                               tr1.data(), size_tr1, tr2_01210.data(),
                               tr2_01210.size(), sampling_freq, xcorr_result)

  );
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 1.0);

  // real-time trace
  const std::array<double, 5> tr2_12100{1, 2, 1, 0, 0};

  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  BOOST_TEST_CHECK(true == template_detail::XCorr(
                               tr1.data(), size_tr1, tr2_12100.data(),
                               tr2_12100.size(), sampling_freq, xcorr_result)

  );
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 0.0);

  // real-time trace
  const std::array<double, 5> tr2_00121{0, 0, 1, 2, 1};

  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  BOOST_TEST_CHECK(true == template_detail::XCorr(
                               tr1.data(), size_tr1, tr2_00121.data(),
                               tr2_00121.size(), sampling_freq, xcorr_result)

  );
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 2.0);

  tr1 = {1, 1, 1};
  // real-time trace
  const std::array<double, 5> tr2_00000{0, 0, 0, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  BOOST_TEST_CHECK(false == template_detail::XCorr(
                                tr1.data(), size_tr1, tr2_00000.data(),
                                tr2_00000.size(), sampling_freq, xcorr_result));

  tr1 = {0, -1, 0};
  // real-time trace
  const std::array<double, 5> tr2_00100{0, 0, 1, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  BOOST_TEST_CHECK(true == template_detail::XCorr(
                               tr1.data(), size_tr1, tr2_00100.data(),
                               tr2_00100.size(), sampling_freq, xcorr_result));
  BOOST_TEST_CHECK(xcorr_result->coefficient == 0.5);
  BOOST_TEST_CHECK(xcorr_result->lag == 0.0);

  tr1 = {2, 4, 2};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  BOOST_TEST_CHECK(true == template_detail::XCorr(
                               tr1.data(), size_tr1, tr2_01210.data(),
                               tr2_01210.size(), sampling_freq, xcorr_result));
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 1.0);

  tr1 = {0, 0, 1};
  // real-time trace
  const std::array<double, 5> tr2_10000{1, 0, 0, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  BOOST_TEST_CHECK(true == template_detail::XCorr(
                               tr1.data(), size_tr1, tr2_10000.data(),
                               tr2_10000.size(), sampling_freq, xcorr_result));
  BOOST_TEST_CHECK(xcorr_result->coefficient == -0.5);
  BOOST_TEST_CHECK(xcorr_result->lag == 0.0);
}

} // namespace detect
} // namespace Seiscomp
