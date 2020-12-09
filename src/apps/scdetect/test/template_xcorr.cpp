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
  // TODO(damb): To be clarified with luca-s.
  constexpr auto max_lag_samples{2};

  // template waveform trace
  constexpr auto size_tr1{3};
  std::array<double, size_tr1> tr1{1, 2, 1};
  auto sum_tr1{SumTr1(tr1)};
  auto squared_sum_tr1{SquaredSumTr1(tr1)};

  // real-time trace
  constexpr auto size_tr2{5};
  std::array<double, size_tr2> tr2{0, 1, 2, 1, 0};

  auto xcorr_result{utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{})};

  bool retval{};
  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 1.0);

  tr1 = {1, 1, 1};
  tr2 = {0, 0, 0, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == false);
  BOOST_TEST_CHECK(xcorr_result->coefficient == 0.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 0.0);

  tr1 = {0, -1, 0};
  tr2 = {0, 0, 1, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == -1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 1.0);

  tr1 = {2, 4, 2};
  tr2 = {0, 1, 2, 1, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 1.0);

  tr1 = {0, 0, 1};
  tr2 = {1, 0, 0, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == -2.0);

  tr1 = {0, 0, 1};
  tr2 = {1, 0, 0, 0, 0};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == -2.0);

  tr1 = {0, 1, 0};
  tr2 = {0, 0, 0, 0, 1};
  sum_tr1 = SumTr1(tr1);
  squared_sum_tr1 = SquaredSumTr1(tr1);
  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, max_lag_samples, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == -0.5);
  BOOST_TEST_CHECK(xcorr_result->lag == 2.0);

  xcorr_result = utils::make_smart<Template::MatchResult>(
      sum_tr1, squared_sum_tr1, size_tr1, Template::MatchResult::MetaData{});

  // TODO(damb): To be clarified with luca-s.
  retval = template_detail::XCorr(tr1.data(), size_tr1, tr2.data(), size_tr2,
                                  sampling_freq, 4, xcorr_result);

  BOOST_TEST_CHECK(retval == true);
  BOOST_TEST_CHECK(xcorr_result->coefficient == 1.0);
  BOOST_TEST_CHECK(xcorr_result->lag == 3.0);
}

} // namespace detect
} // namespace Seiscomp
