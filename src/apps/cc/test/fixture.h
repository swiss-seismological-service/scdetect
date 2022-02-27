#ifndef SCDETECT_APPS_CC_TEST_FIXTURE_H_
#define SCDETECT_APPS_CC_TEST_FIXTURE_H_

namespace Seiscomp {
namespace detect {
namespace test {

// Fixture implementing CLI parsing facilities
struct CLIParserFixture {
  CLIParserFixture() = default;
  ~CLIParserFixture() = default;

  void setup();
  void teardown();

  static bool keepTempdir;
};

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_TEST_FIXTURE_H_
