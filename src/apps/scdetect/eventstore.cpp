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
    const boost::optional<size_t> &buffer_size)
    : PublicObjectCache{archive}, buffer_size_{buffer_size} {}

void PublicObjectBuffer::set_buffer_size(
    const boost::optional<size_t> &buffer_size) {
  buffer_size_ = buffer_size;
}

boost::optional<size_t> PublicObjectBuffer::buffer_size() const {
  return buffer_size_;
}

bool PublicObjectBuffer::feed(DataModel::PublicObject *po) {
  push(po);
  if (buffer_size_) {
    while (size() > buffer_size_) pop();
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

const int EventStore::buffer_size_{25000};

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

void EventStore::Load(const std::string &path) {
  Load(LoadXMLArchive(path).get());
}
void EventStore::Load(const boost::filesystem::path &path) {
  Load(path.string());
}

void EventStore::Load(DataModel::EventParameters *ep) {
  auto db_query{
      utils::make_smart<DataModel::DatabaseQuery>(CreateInMemoryDB(ep).get())};
  Load(db_query.get());
}

void EventStore::Load(DataModel::DatabaseQuery *db_query) {
  Reset();
  cache_.setDatabaseArchive(db_query);
  db_query_ = db_query;
}

void EventStore::Reset() {
  cache_.clear();
  cache_.setDatabaseArchive(nullptr);
  db_query_.reset();
}

DataModel::EventPtr EventStore::GetEvent(const std::string &origin_id) const {
  auto event{db_query_->getEvent(origin_id)};
  if (event) {
    cache_.feed(event);
    return event;
  }
  return nullptr;
}

DataModel::PublicObject *EventStore::Get(const Core::RTTI &class_type,
                                         const std::string &public_id,
                                         bool loadChildren) const {
  auto retval{cache_.find(class_type, public_id, loadChildren)};
  if (retval) {
    return retval;
  }
  return nullptr;
}

DataModel::EventParametersPtr EventStore::LoadXMLArchive(
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

IO::DatabaseInterfacePtr EventStore::CreateInMemoryDB(
    DataModel::EventParameters *ep) {
  IO::DatabaseInterfacePtr db_engine_write{
      IO::DatabaseInterface::Open("sqlite3_://:memory:")};
  if (!db_engine_write) {
    throw EventStore::DatabaseException{
        "Failed to initialize SQLite in-memory DB"};
  }
  DataModel::createAll(db_engine_write.get());
  DataModel::DatabaseArchive db_archive{db_engine_write.get()};
  DataModel::DatabaseObjectWriter writer{db_archive};
  writer(ep);

  // XXX(damb): Create a separate interface - `db_engine_write` is going to be
  // closed by the `db_archive` instance when going out of scope.
  IO::DatabaseInterfacePtr db_engine_read{db_engine_write};
  return db_engine_read;
}

}  // namespace detect
}  // namespace Seiscomp
