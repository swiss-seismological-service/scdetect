#include "eventstore.h"

#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/databasereader.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/publicobject.h>
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/io/database.h>

#include <vector>

#include "datamodel/ddl.h"
#include "log.h"
#include "utils.h"

namespace Seiscomp {
namespace detect {

namespace detail {

PublicObjectBuffer::PublicObjectBuffer() {}
PublicObjectBuffer::PublicObjectBuffer(
    DataModel::DatabaseReader *archive,
    const boost::optional<size_t> &bufferSize)
    : PublicObjectCache{archive}, _bufferSize{bufferSize} {}

void PublicObjectBuffer::setBufferSize(
    const boost::optional<size_t> &bufferSize) {
  _bufferSize = bufferSize;
}

boost::optional<size_t> PublicObjectBuffer::bufferSize() const {
  return _bufferSize;
}

bool PublicObjectBuffer::feed(DataModel::PublicObject *po) {
  push(po);
  if (_bufferSize) {
    while (size() > _bufferSize) pop();
  }
  return true;
}

DataModel::PublicObject *PublicObjectBuffer::find(const Core::RTTI &classType,
                                                  const std::string &publicID,
                                                  bool loadChildren) {
  if (loadChildren) {
    bool cached{true};
    auto po{DataModel::PublicObject::Find(publicID)};
    if (!po) {
      cached = false;
      po = databaseArchive()
               ? dynamic_cast<DataModel::DatabaseReader *>(databaseArchive())
                     ->loadObject(classType, publicID)
               : nullptr;
    }
    setCached(cached);
    if (po) {
      // XXX(damb): Currently, the requested object is cached, only. That is,
      // children are not fed.
      feed(po);
    }
    return po;
  }
  return PublicObjectCache::find(classType, publicID);
}

}  // namespace detail

const int EventStore::_bufferSize{25000};

EventStore::BaseException::BaseException()
    : Exception{"base EventStore exception"} {}
EventStore::SCMLException::SCMLException() : BaseException{"SCML exception"} {}
EventStore::DatabaseException::DatabaseException()
    : BaseException{"database exception"} {}

EventStore &EventStore::Instance() {
  // guaranteed to be destroyed; instantiated on first use
  static EventStore instance;
  return instance;
}

void EventStore::load(const std::string &path) {
  load(loadXMLArchive(path).get());
}
void EventStore::load(const boost::filesystem::path &path) {
  load(path.string());
}

void EventStore::load(DataModel::EventParameters *ep) {
  auto db_query{
      utils::make_smart<DataModel::DatabaseQuery>(createInMemoryDb(ep).get())};
  load(db_query.get());
}

void EventStore::load(DataModel::DatabaseQuery *db) {
  reset();
  _cache.setDatabaseArchive(db);
  _dbQuery = db;
}

void EventStore::reset() {
  _cache.clear();
  _cache.setDatabaseArchive(nullptr);
  _dbQuery.reset();
}

DataModel::EventPtr EventStore::getEvent(const std::string &originId) const {
  auto event{_dbQuery->getEvent(originId)};
  if (event) {
    _cache.feed(event);
    return event;
  }
  return nullptr;
}

DataModel::PublicObject *EventStore::get(const Core::RTTI &classType,
                                         const std::string &publicId,
                                         bool loadChildren) const {
  auto retval{_cache.find(classType, publicId, loadChildren)};
  if (retval) {
    return retval;
  }
  return nullptr;
}

DataModel::EventParametersPtr EventStore::loadXMLArchive(
    const std::string &path) {
  DataModel::EventParametersPtr ep;
  if (!path.empty()) {
    IO::XMLArchive ar;
    if (!ar.open(path.c_str())) {
      throw SCMLException{std::string("Failed to open file: ") + path};
    }
    ar >> ep;
    ar.close();
  }
  return ep;
}

IO::DatabaseInterfacePtr EventStore::createInMemoryDb(
    DataModel::EventParameters *ep) {
  IO::DatabaseInterfacePtr dbEngineWrite{
      IO::DatabaseInterface::Open("sqlite3_://:memory:")};
  if (!dbEngineWrite) {
    throw EventStore::DatabaseException{
        "Failed to initialize SQLite in-memory DB"};
  }
  DataModel::createAll(dbEngineWrite.get());
  DataModel::DatabaseArchive dbArchive{dbEngineWrite.get()};
  DataModel::DatabaseObjectWriter writer{dbArchive};
  writer(ep);

  // XXX(damb): Create a separate interface - `dbEngineWrite` is going to be
  // closed by the `dbArchive` instance when going out of scope.
  IO::DatabaseInterfacePtr dbEngineRead{dbEngineWrite};
  return dbEngineRead;
}

}  // namespace detect
}  // namespace Seiscomp
