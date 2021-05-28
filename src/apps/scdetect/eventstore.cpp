#include "eventstore.h"

#include <vector>

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/databasequery.h>
#include <seiscomp/datamodel/databasereader.h>
#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/publicobject.h>
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/io/database.h>

#include "datamodel/ddl.h"
#include "log.h"
#include "seiscomp/datamodel/databasequery.h"
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
    while (size() > buffer_size_)
      pop();
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

} // namespace detail

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
  DataModel::EventParametersPtr ep;
  LoadXMLArchive(path, ep);
  Load(ep);
}
void EventStore::Load(const boost::filesystem::path &path) {
  Instance().Load(path.string());
}

void EventStore::Load(DataModel::EventParametersPtr &ep) {
  auto db_query{CreateInMemoryDB(ep)};
  Load(db_query);
}

void EventStore::Load(DataModel::DatabaseQueryPtr db) {
  Reset();
  cache_.setDatabaseArchive(db.get());
  db_ = db;
}

void EventStore::Reset() {
  cache_.clear();
  cache_.setDatabaseArchive(nullptr);
  db_.reset();
}

DataModel::EventPtr EventStore::GetEvent(const std::string &origin_id) const {
  auto event{db_->getEvent(origin_id)};
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

void EventStore::LoadXMLArchive(const std::string &path,
                                DataModel::EventParametersPtr &ep) {
  if (!path.empty()) {
    IO::XMLArchive ar;
    if (!ar.open(path.c_str())) {
      throw SCMLException{std::string("Failed to open file: ") + path};
    }
    ar >> ep;
    ar.close();
  }
}

DataModel::DatabaseQueryPtr
EventStore::CreateInMemoryDB(DataModel::EventParametersPtr &ep) {
  IO::DatabaseInterfacePtr db_engine{
      IO::DatabaseInterface::Open("sqlite3://:memory:")};
  if (!db_engine) {
    throw EventStore::DatabaseException{
        "Failed to initialize SQLite in-memory DB"};
  }
  DataModel::createAll(db_engine.get());
  DataModel::DatabaseArchive db_archive{db_engine.get()};
  DataModel::DatabaseObjectWriter writer{db_archive};

  writer(ep.get());
  return utils::make_smart<DataModel::DatabaseQuery>(db_engine.get());
}

} // namespace detect
} // namespace Seiscomp
