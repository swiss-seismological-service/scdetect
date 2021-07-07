#define SEISCOMP_TEST_MODULE test_foo
#include <seiscomp/unittest/unittests.h>
/* #define BOOST_TEST_MODULE test_foo */
/* #include <boost/test/included/unit_test.hpp> */

#include <seiscomp/client/streamapplication.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/test/data/test_case.hpp>
#include <ostream>
#include <vector>

#include "../config.h"
#include "../detector.h"
#include "../eventstore.h"
#include "../log.h"

namespace utf = boost::unit_test;
namespace fs = boost::filesystem;

namespace Seiscomp {
namespace myapp {

class MyApp : public Client::StreamApplication {
 public:
  MyApp(int argc, char **argv) : StreamApplication(argc, argv) {
    BOOST_TEST_MESSAGE("MyApp ctor called");

    setLoadStationsEnabled(true);
    setLoadInventoryEnabled(true);
    setMessagingEnabled(false);

    /* setPrimaryMessagingGroup("LOCATION"); */
  }
  ~MyApp() override { BOOST_TEST_MESSAGE("MyApp dtor called"); }

  struct Config {
    std::string path_event_db;

    std::string path_templates_json;

    detect::DetectorConfig detector_config;
    detect::StreamConfig stream_config;
  };

 protected:
  void handleRecord(Record *rec) override {
    if (!rec->data()) {
      return;
    }

    if (!record_received_) {
      SEISCOMP_DEBUG("First record received: stream=%s, start=%s, end=%s",
                     rec->streamID().c_str(), rec->startTime().iso().c_str(),
                     rec->endTime().iso().c_str());
      record_received_ = true;
    }
  }

  void createCommandLineDescription() override {
    StreamApplication::createCommandLineDescription();

    commandline().addOption("Database", "event-db",
                            "File URI to event database",
                            &config_.path_event_db);
    commandline().addGroup("Input");
    commandline().addOption("Input", "templates-json",
                            "Path to templates configuration file (JSON)",
                            &config_.path_templates_json);
  }

  bool validateParameters() override {
    if (!StreamApplication::validateParameters()) return false;

    if (config_.path_event_db.empty()) {
      return false;
    } else if (0 != config_.path_event_db.find("file://")) {
      return false;
    } else {
      config_.path_event_db = config_.path_event_db.substr(7);
    }

    if (config_.path_templates_json.empty()) {
      return false;
    }

    return true;
  }

  bool init() override {
    bool retval{StreamApplication::init()};

    SEISCOMP_INFO("Initializing MyApp ...");
    SEISCOMP_DEBUG("Loading events from: %s", config_.path_event_db.c_str());
    detect::EventStore::Instance().Load(config_.path_event_db);

    if (!detect::EventStore::Instance().event_parameters()) {
      SEISCOMP_ERROR("No event parameters found.");
      return false;
    }

    detect::WaveformHandlerIfacePtr waveform_handler{
        detect::utils::make_smart<detect::WaveformHandler>(recordStreamURL())};

    std::ifstream ifs{config_.path_templates_json};
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ifs, pt);

    /* for (const auto &template_setting_pt : pt) { */
    /*   detect::TemplateConfig tc{template_setting_pt.second, */
    /*                             config_.detector_config,
     * config_.stream_config}; */

    /*   SCDETECT_LOG_DEBUG("Creating detector processor (id=%s) ... ", */
    /*                      tc.detector_id().c_str()); */

    /*   auto detector_builder{ */
    /*       detect::Detector::Create(tc.detector_id(), tc.origin_id()) */
    /*           .set_config(tc.detector_config()) */
    /*           .set_eventparameters()}; */

    /*   for (const auto &stream_set : tc) { */
    /*     for (const auto &stream_config_pair : stream_set) { */
    /*       detector_builder.set_stream(stream_config_pair.first, */
    /*                                   stream_config_pair.second, */
    /*                                   waveform_handler); */
    /*     } */
    /*   } */

    /*   auto detector{detector_builder.build()}; */
    /*   detectors_.push_back(detector); */
    /* } */

    return retval;
  }

  bool run() override {
    SEISCOMP_DEBUG("Running MyApp ...");

    // subscribe to stream
    recordStream()->addStream("CH", "GRIMS", "", "HHZ");

    return StreamApplication::run();
  }

  void done() override {
    StreamApplication::done();

    detect::EventStore::Instance().Reset();
    SEISCOMP_DEBUG("MyApp done.");
  }

 private:
  bool record_received_{false};

  Config config_;

  std::vector<detect::DetectorPtr> detectors_;
};

struct Sample {
  std::vector<std::string> argv;

  friend std::ostream &operator<<(std::ostream &os, const Sample &s) {
    return os << boost::algorithm::join(s.argv, " ");
  }
};

std::vector<Sample> dataset{Sample{
    {"myapp", "--debug", "--author=foo", "--agencyID=foo",
     "--record-url=file:///home/damb/work/projects/seiscomp/src/extras/"
     "scdetect/src/apps/scdetect/test/data/"
     "myapp/data.mseed",
     "--inventory-db=file:///home/damb/work/projects/seiscomp/src/"
     "extras/scdetect/src/apps/scdetect/test/data/"
     "myapp/inventory.scml",
     "--plugins=dbsqlite3",
     "--database=sqlite3:///home/damb/work/projects/seiscomp/src/"
     "extras/scdetect/src/apps/scdetect/test/data/myapp/seiscomp_db.sqlite",
     "--event-db=file:///home/damb/work/projects/seiscomp/src/"
     "extras/scdetect/src/apps/scdetect/test/data/myapp/catalog.scml",
     "--templates-json=/home/damb/work/projects/seiscomp/src/"
     "extras/scdetect/src/apps/scdetect/test/data/myapp/templates.json"}}};
/* Sample{{"myapp", "--debug", "--author=bar", "--agencyID=bar", */
/*         "--record-url=file:///home/damb/work/projects/seiscomp/src/extras/"
 */
/*         "scdetect/src/apps/scdetect/test/data/" */
/*         "integration-single-stream-simple/data.mseed", */
/*         "--inventory-db=file:///home/damb/work/projects/seiscomp/src/" */
/*         "extras/scdetect/src/apps/scdetect/test/data/" */
/*         "integration-single-stream-simple/inventory.scml", */
/*         "--plugins=dbsqlite3", */
/*         "--database=sqlite3:///tmp/scdetect/seiscomp_db.sqlite", */
/* "--event-db=file:///tmp/scdetect/catalog.scml"}}}; */

/* BOOST_DATA_TEST_CASE(myapp, dataset) { */
BOOST_AUTO_TEST_CASE(myapp) {
  int retval{EXIT_SUCCESS};

  /* auto StrToCStr = [](const std::string &str) { */
  /*   char *ret{new char[str.size() + 1]}; */
  /*   std::strcpy(ret, str.c_str()); */
  /*   return ret; */
  /* }; */

  /* std::vector<char *> argv_cstr; */
  /* argv_cstr.reserve(sample.argv.size()); */
  /* std::transform(sample.argv.cbegin(), sample.argv.cend(), */
  /*                back_inserter(argv_cstr), StrToCStr); */

  /* retval = MyApp{static_cast<int>(argv_cstr.size()), argv_cstr.data()}(); */
  BOOST_TEST_CHECK(EXIT_SUCCESS == retval);

  // perform additional checks
  // ...
  //
  /* for (size_t i = 0; i < argv_cstr.size(); ++i) { */
  /*   delete[] argv_cstr[i]; */
  /* } */
}

}  // namespace myapp
}  // namespace Seiscomp
