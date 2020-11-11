#include "eventstore.h"

#include <seiscomp/datamodel/event.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/io/archive/xmlarchive.h>

#include "version.h"

namespace Seiscomp {
namespace detect {

EventStore &EventStore::Instance() {
  // guaranteed to be destroyed; instantiated on first use
  static EventStore instance;
  return instance;
}

void EventStore::Load(const std::string &path) {
  if (!path.empty()) {
    IO::XMLArchive ar;
    if (!ar.open(path.c_str())) {
      throw SCMLException{std::string("Failed to open file: ") + path};
    }
    ar >> event_parameters_;
    ar.close();
  }

  cache_.clear();
  // populate cache
  if (!set_cache(event_parameters_)) {
    cache_.clear();
  }
}

void EventStore::Load(const boost::filesystem::path &path) {
  Load(path.string());
}

void EventStore::Load(DataModel::EventParametersPtr ep) {
  event_parameters_ = ep;

  cache_.clear();
  // populate cache
  if (!set_cache(event_parameters_)) {
    cache_.clear();
  }
}

void EventStore::Load(DataModel::DatabaseReaderPtr db) {

  if (db) {
    event_parameters_ = db->loadEventParameters();
  }

  cache_.clear();
  // populate cache
  if (!set_cache(event_parameters_)) {
    cache_.clear();
  }
}

EventStore::SCMLException::SCMLException() : Exception{"SCML exception"} {}

DataModel::EventParametersCPtr EventStore::event_parameters() const {
  return event_parameters_;
}

bool EventStore::set_cache(DataModel::EventParametersPtr ep) {
  if (!ep || !cache_.feed(ep.get()))
    return false;

  // load events
  for (size_t i = 0; i < ep->eventCount(); ++i) {
    if (!cache_.feed(ep->event(i))) {
      return false;
    }
  }

  // load origins
  for (size_t i = 0; i < ep->originCount(); ++i) {

    DataModel::Origin *origin{ep->origin(i)};
    if (!cache_.feed(origin)) {
      return false;
    }

    // load magnitudes
    for (size_t j = 0; j < origin->magnitudeCount(); ++j) {
      if (!cache_.feed(origin->magnitude(j))) {
        return false;
      }
    }
  }

  // load picks
  for (size_t i = 0; i < ep->pickCount(); ++i) {
    if (!cache_.feed(ep->pick(i))) {
      return false;
    }
  }

  return true;
}

} // namespace detect
} // namespace Seiscomp
