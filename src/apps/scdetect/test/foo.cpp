#define BOOST_TEST_MODULE test_foo
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(test_foo) {
  BOOST_TEST_MESSAGE("Boost version: " << BOOST_VERSION);

  BOOST_TEST_CHECK(true);
}
