#ifndef SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_IPP_
#define SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_IPP_

#include <algorithm>
#include <iterator>

template <typename TApp>
ApplicationWrapper<TApp>::ApplicationWrapper(
    const std::vector<std::string> &argv) {
  auto StrToCStr = [](const std::string &str) {
    char *ret{new char[str.size() + 1]};
    std::strcpy(ret, str.c_str());
    return ret;
  };

  argv_.reserve(argv.size());
  std::transform(argv.cbegin(), argv.cend(), back_inserter(argv_), StrToCStr);
}

template <typename TApp> ApplicationWrapper<TApp>::~ApplicationWrapper() {
  for (size_t i = 0; i < argv_.size(); ++i) {
    delete[] argv_[i];
  }
}

template <typename TApp> int ApplicationWrapper<TApp>::operator()() {
  return TApp(static_cast<int>(argv_.size()), argv_.data())();
}

#endif // SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_IPP_
