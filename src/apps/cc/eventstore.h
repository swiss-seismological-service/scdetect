#ifndef SCDETECT_APPS_CC_EVENTSTORE_H_
#define SCDETECT_APPS_CC_EVENTSTORE_H_

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
                     const boost::optional<size_t> &bufferSize);

  void setBufferSize(const boost::optional<size_t> &bufferSize);
  boost::optional<size_t> bufferSize() const;

  bool feed(DataModel::PublicObject *po) override;

  DataModel::PublicObject *find(const Core::RTTI &classType,
                                const std::string &publicID, bool loadChildren);

 private:
  boost::optional<size_t> _bufferSize;
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
#if SC_API_VERSION < SC_API_VERSION_CHECK(17, 0, 0)
  using SmartPointer = typename Core::SmartPointer<T>::Impl;
#else
  using SmartPointer = Core::SmartPointer<T>;
#endif

  static EventStore &Instance();

  EventStore(const EventStore &) = delete;
  EventStore &operator=(const EventStore &) = delete;

  void load(const std::string &path);
  void load(const boost::filesystem::path &path);
  void load(DataModel::EventParameters *ep);
  void load(DataModel::DatabaseQuery *db);

  // Reset the store
  void reset();

  // Returns the requested object specified by `publicId` (excluding
  // descendants)
  template <typename T>
  SmartPointer<T> get(const std::string &publicId) const {
    return T::Cast(get(T::TypeInfo(), publicId));
  }

  // Returns the requested object specified by `publicId` (including
  // descendants)
  template <typename T>
  SmartPointer<T> getWithChildren(const std::string &publicId) const {
    return T::Cast(get(T::TypeInfo(), publicId, true));
  }

  // Returns the event for a given `originId` if any
  DataModel::EventPtr getEvent(const std::string &originId) const;

 protected:
  DataModel::PublicObject *get(const Core::RTTI &classType,
                               const std::string &publicId,
                               bool loadChildren = false) const;

  DataModel::EventParametersPtr loadXMLArchive(const std::string &path);

  // Create an in-memory SQLite DB populated with `ep` and return the
  // corresponding pointer to the database engine created
  IO::DatabaseInterfacePtr createInMemoryDb(DataModel::EventParameters *ep);

 private:
  EventStore() {}

  DataModel::DatabaseQueryPtr _dbQuery;
  mutable detail::PublicObjectBuffer _cache;

  static const int _bufferSize;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_EVENTSTORE_H_
