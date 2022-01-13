#ifndef SCDETECT_APPS_SCDETECT_FACTORY_H_
#define SCDETECT_APPS_SCDETECT_FACTORY_H_

#include <functional>
#include <map>
#include <memory>
#include <utility>

namespace Seiscomp {
namespace detect {

template <typename TAbstractProduct, typename TIdentifierType, typename... Args>
class Factory {
 public:
  using CallbackType =
      std::function<std::unique_ptr<TAbstractProduct>(Args &&...)>;

  // Register a factory method
  static bool registerFactory(const TIdentifierType &id,
                              CallbackType callback) {
    return callbackMap().emplace(std::make_pair(id, callback)).second;
  }

  // Unregister a registered factory method
  static bool unregisterFactory(const TIdentifierType &id) {
    return callbackMap().erase(id) == 1;
  }

  // Creates an object
  static std::unique_ptr<TAbstractProduct> create(const TIdentifierType &id,
                                                  Args &&...args) {
    auto it{callbackMap().find(id)};
    if (it != callbackMap().end()) {
      return (it->second)(std::forward<Args>(args)...);
    }

    return nullptr;
  }

 protected:
  // Resets the factory callbacks
  static void resetCallbacks() { callbackMap().clear(); }

 private:
  using CallbackMap = std::map<TIdentifierType, CallbackType>;
  // prevent the static order initialization problem
  static CallbackMap &callbackMap() {
    static CallbackMap *ret{new CallbackMap{}};
    return *ret;
  }
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_FACTORY_H_
