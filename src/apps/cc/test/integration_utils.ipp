#ifndef SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_IPP_
#define SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_IPP_

namespace Seiscomp {
namespace detect {
namespace test {

#include <algorithm>
#include <iterator>

template <typename TApp>
ApplicationWrapper<TApp>::ApplicationWrapper(
    const std::vector<std::string> &argv) {
  auto strToCStr = [](const std::string &str) {
    char *ret{new char[str.size() + 1]};
    std::strcpy(ret, str.c_str());
    return ret;
  };

  _argv.reserve(argv.size());
  std::transform(argv.cbegin(), argv.cend(), back_inserter(_argv), strToCStr);
}

template <typename TApp>
ApplicationWrapper<TApp>::~ApplicationWrapper() {
  for (size_t i = 0; i < _argv.size(); ++i) {
    delete[] _argv[i];
  }
}

template <typename TApp>
int ApplicationWrapper<TApp>::operator()() {
  return TApp(static_cast<int>(_argv.size()), _argv.data())();
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_IPP_
