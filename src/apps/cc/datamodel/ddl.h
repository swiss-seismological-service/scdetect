#ifndef SCDETECT_APPS_SCDETECT_DATAMODEL_DDL_H_
#define SCDETECT_APPS_SCDETECT_DATAMODEL_DDL_H_

#include <seiscomp/io/database.h>

namespace Seiscomp {
namespace DataModel {

// Create the database schema. Tables already present are dropped.
void createAll(IO::DatabaseInterface *dbDriver);

}  // namespace DataModel
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DATAMODEL_DDL_H_
