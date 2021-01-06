#ifndef SCDETECT_APPS_SCDETECT_EVENTSTORE_H_
#define SCDETECT_APPS_SCDETECT_EVENTSTORE_H_

#include <boost/filesystem.hpp>

#include <seiscomp/core/defs.h>
#include <seiscomp/datamodel/databasereader.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/publicobjectcache.h>

#include "exception.h"

namespace Seiscomp {
namespace detect {

// Implements the Singleton Design Pattern
class EventStore {

public:
  template <typename T>
  using SmartPointer = typename Core::SmartPointer<T>::Impl;

  static EventStore &Instance();

  EventStore(const EventStore &) = delete;
  void operator=(const EventStore &) = delete;

  void Load(const std::string &path);
  void Load(const boost::filesystem::path &path);
  void Load(DataModel::EventParametersPtr ep);
  void Load(DataModel::DatabaseReaderPtr db);

  // Reset the store
  void Reset();

  DataModel::EventParametersCPtr event_parameters() const;

  template <typename T> SmartPointer<T> Get(const std::string &public_id) {
    return T::Cast(Get(T::TypeInfo(), public_id));
  }

  class SCMLException : public Exception {
  public:
    using Exception::Exception;
    SCMLException();
  };

protected:
  // Populate the cache from event parameters.
  virtual bool set_cache(DataModel::EventParametersPtr ep);

  DataModel::PublicObject *Get(const Core::RTTI &class_type,
                               const std::string &public_id);

private:
  EventStore() {}

  DataModel::EventParametersPtr event_parameters_{nullptr};
  DataModel::PublicObjectTimeSpanBuffer cache_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_EVENTSTORE_H_
