#ifndef SCDETECT_APPS_SCDETECT_TEST_FIXTURE_H_
#define SCDETECT_APPS_SCDETECT_TEST_FIXTURE_H_

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

// Fixture implementing CLI parsing facilities
struct CLIParserFixture {
  CLIParserFixture();
  ~CLIParserFixture();

  void setup();
  void teardown();

  static fs::path pathData;
  static bool keepTempdir;
};

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEST_FIXTURE_H_
