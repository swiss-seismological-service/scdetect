#include "fixture.h"

#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>

namespace po = boost::program_options;
namespace utf = boost::unit_test;

namespace Seiscomp {
namespace detect {
namespace test {

bool CLIParserFixture::keepTempdir{false};

void CLIParserFixture::setup() {
  try {
    po::options_description desc;
    desc.add_options()("keep-tempfiles",
                       po::value<bool>(&keepTempdir)->default_value(false),
                       "Keep temporary files from tests");

    po::variables_map vm;
    po::store(po::command_line_parser(utf::framework::master_test_suite().argc,
                                      utf::framework::master_test_suite().argv)
                  .options(desc)
                  .allow_unregistered()
                  .run(),
              vm);
    po::notify(vm);
  } catch (std::exception &e) {
    BOOST_TEST_FAIL(e.what());
  }
}

void CLIParserFixture::teardown() {}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
