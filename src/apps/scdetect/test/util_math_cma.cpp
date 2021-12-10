#define SEISCOMP_TEST_MODULE test_util_math_cma

#include <seiscomp/unittest/unittests.h>

#include <array>

#include "../util/math.h"

namespace utf = boost::unit_test;

constexpr double testUnitTolerance{0.000001};

namespace Seiscomp {
namespace detect {

BOOST_AUTO_TEST_CASE(cma, *utf::tolerance(testUnitTolerance)) {
  std::array<double, 10> samples0123456789{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  BOOST_TEST_CHECK(
      4.5 == util::cma(samples0123456789.data(), samples0123456789.size()));

  std::array<double, 5> samples00100{0, 0, 1, 0, 0};
  BOOST_TEST_CHECK(0.2 == util::cma(samples00100.data(), samples00100.size()));

  std::array<double, 4> samples_111_1{-1, 1, 1, -1};
  BOOST_TEST_CHECK(0.0 ==
                   util::cma(samples_111_1.data(), samples_111_1.size()));

  std::array<double, 3> samples000{0, 0, 0};
  BOOST_TEST_CHECK(0.0 == util::cma(samples000.data(), samples000.size()));
}

}  // namespace detect
}  // namespace Seiscomp
