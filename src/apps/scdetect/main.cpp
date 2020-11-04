#include <cstdlib>

#include "app.h"

int main(int argc, char **argv) {
  int retCode = EXIT_SUCCESS;

  // Create an own block to make sure the application object
  // is destroyed when printing the overall objectcount
  {
    Seiscomp::detect::Application app(argc, argv);
    retCode = app.exec();
  }

  SEISCOMP_DEBUG("EXIT(%d), remaining objects: %d", retCode,
                 Seiscomp::Core::BaseObject::ObjectCount());

  return retCode;
}
