#include <cstdlib>

#include "app.h"
#include "log.h"

int main(int argc, char **argv) {
  int code{EXIT_SUCCESS};

  // Create an own block to make sure the application object
  // is destroyed when printing the overall object count.
  { code = Seiscomp::detect::Application{argc, argv}(); }

  SCDETECT_LOG_DEBUG("EXIT(%d), remaining objects: %d", code,
                     Seiscomp::Core::BaseObject::ObjectCount());

  return code;
}
