#ifndef SCDETECT_APPS_SCDETECT_EVENTSTORE_H_
#define SCDETECT_APPS_SCDETECT_EVENTSTORE_H_

#include <seiscomp/core/defs.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/databasereader.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/publicobject.h>
#include <seiscomp/datamodel/publicobjectcache.h>
#include <seiscomp/io/database.h>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "exception.h"

namespace Seiscomp {
namespace detect {

namespace detail {
class PublicObjectBuffer : public DataModel::PublicObjectCache {
 public:
  PublicObjectBuffer();
  PublicObjectBuffer(DataModel::DatabaseReader *archive,
                     const boost::optional<size_t> &buffer_size);

  void set_buffer_size(const boost::optional<size_t> &buffer_size);
  boost::optional<size_t> buffer_size() const;

  bool feed(DataModel::PublicObject *po) override;

  DataModel::PublicObject *find(const Core::RTTI &classType,
                                const std::string &publicID, bool loadChildren);

 private:
  boost::optional<size_t> buffer_size_;
};

}  // namespace detail

// An utility interface to access event parameters
// - implements the Singleton Design Pattern
class EventStore {
 public:
  class BaseException : public Exception {
   public:
    using Exception::Exception;
    BaseException();
  };

  class SCMLException : public BaseException {
   public:
    using BaseException::BaseException;
    SCMLException();
  };

  class DatabaseException : public BaseException {
   public:
    using BaseException::BaseException;
    DatabaseException();
  };

  template <typename T>
  using SmartPointer = typename Core::SmartPointer<T>::Impl;

  static EventStore &Instance();

  EventStore(const EventStore &) = delete;
  void operator=(const EventStore &) = delete;

  void Load(const std::string &path);
  void Load(const boost::filesystem::path &path);
  void Load(DataModel::EventParameters *ep);
  void Load(DataModel::DatabaseQuery *db);

  // Reset the store
  void Reset();

  // Returns the requested object specified by `public_id` (excluding
  // descendants)
  template <typename T>
  SmartPointer<T> Get(const std::string &public_id) const {
    return T::Cast(Get(T::TypeInfo(), public_id));
  }

  // Returns the requested object specified by `public_id` (including
  // descendants)
  template <typename T>
  SmartPointer<T> GetWithChildren(const std::string &public_id) const {
    return T::Cast(Get(T::TypeInfo(), public_id, true));
  }

  // Returns the event for a given `origin_id` if any
  DataModel::EventPtr GetEvent(const std::string &origin_id) const;

 protected:
  DataModel::PublicObject *Get(const Core::RTTI &class_type,
                               const std::string &public_id,
                               bool loadChildren = false) const;

  DataModel::EventParametersPtr LoadXMLArchive(const std::string &path);

  // Create an in-memory SQLite DB populated with `ep` and return the
  // corresponding pointer to the database engine created
  IO::DatabaseInterfacePtr CreateInMemoryDB(DataModel::EventParameters *ep);

 private:
  EventStore() {}

  DataModel::DatabaseQueryPtr db_query_;
  mutable detail::PublicObjectBuffer cache_;

  static const int buffer_size_;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_EVENTSTORE_H_
