/***************************************************************************
 * Copyright (C) gempa GmbH                                                *
 * All rights reserved.                                                    *
 * Contact: gempa GmbH (seiscomp-dev@gempa.de)                             *
 *                                                                         *
 * Author: Jan Becker                                                      *
 * Email: jabe@gempa.de                                                    *
 *                                                                         *
 * GNU Affero General Public License Usage                                 *
 * This file may be used under the terms of the GNU Affero                 *
 * Public License version 3.0 as published by the Free Software Foundation *
 * and appearing in the file LICENSE included in the packaging of this     *
 * file. Please review the following information to ensure the GNU Affero  *
 * Public License version 3.0 requirements will be met:                    *
 * https://www.gnu.org/licenses/agpl-3.0.html.                             *
 *                                                                         *
 * Other Usage                                                             *
 * Alternatively, this file may be used in accordance with the terms and   *
 * conditions contained in a signed written agreement between you and      *
 * gempa GmbH.                                                             *
 ***************************************************************************/

#ifndef _SCDETECT_APPS_CC_DETAIL_SQLITE_H_
#define _SCDETECT_APPS_CC_DETAIL_SQLITE_H_

#include <seiscomp/io/database.h>
#include <sqlite3.h>

namespace Seiscomp {
namespace detect {
namespace detail {

class SQLiteDatabase : public Seiscomp::IO::DatabaseInterface {
  DECLARE_SC_CLASS(SQLiteDatabase);
  // ------------------------------------------------------------------
  //  Xstruction
  // ------------------------------------------------------------------
 public:
  SQLiteDatabase();
  ~SQLiteDatabase();

  // ------------------------------------------------------------------
  //  Public interface
  // ------------------------------------------------------------------
 public:
  bool connect(const char *con) override;
  void disconnect() override;

  bool isConnected() const override;

  void start() override;
  void commit() override;
  void rollback() override;

  bool execute(const char *command) override;
  bool beginQuery(const char *query) override;
  void endQuery() override;

  const char *defaultValue() const override;
  OID lastInsertId(const char *) override;
  uint64_t numberOfAffectedRows();

  bool fetchRow() override;
  int findColumn(const char *name) override;
  int getRowFieldCount() const override;
  const char *getRowFieldName(int index) override;
  const void *getRowField(int index) override;
  size_t getRowFieldSize(int index) override;
#if (SC_API_VERSION >= SC_API_VERSION_CHECK(17, 0, 0))
  bool escape(std::string &out, const std::string &in) const override;
#else
  bool escape(std::string &out, const std::string &in) override;
#endif

  // ------------------------------------------------------------------
  //  Protected interface
  // ------------------------------------------------------------------
 protected:
  bool open();

  // ------------------------------------------------------------------
  //  Implementation
  // ------------------------------------------------------------------
 private:
  sqlite3 *_handle;
  sqlite3_stmt *_stmt;
  int _columnCount;
};

}  // namespace detail
}  // namespace detect
}  // namespace Seiscomp

#endif  // _SCDETECT_APPS_CC_DETAIL_SQLITE_H_
