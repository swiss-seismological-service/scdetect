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

#include "sqlite.h"

#include <seiscomp/logging/log.h>
#include <seiscomp/system/environment.h>

#include <cstdio>
#include <cstring>

#include "../log.h"

namespace Seiscomp {
namespace detect {
namespace detail {

IMPLEMENT_SC_CLASS_DERIVED(SQLiteDatabase, Seiscomp::IO::DatabaseInterface,
                           "sqlite3_database_interface_");

REGISTER_DB_INTERFACE(SQLiteDatabase, "sqlite3_");

SQLiteDatabase::SQLiteDatabase()
    : _handle(NULL), _stmt(NULL), _columnCount(0) {}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
SQLiteDatabase::~SQLiteDatabase() { disconnect(); }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::open() {
  std::string uri(_host);
  if (uri != ":memory:") {
    uri = Environment::Instance()->absolutePath(_host);

    FILE *fp = fopen(uri.c_str(), "rb");
    if (fp == NULL) {
      SEISCOMP_ERROR("databasefile '%s' not found", uri.c_str());
      return false;
    }
    fclose(fp);
  }

  int res = sqlite3_open(uri.c_str(), &_handle);
  if (res != SQLITE_OK) {
    SEISCOMP_ERROR("sqlite3 open error: %d", res);
    sqlite3_close(_handle);
    return false;
  }

  return true;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::connect(const char *con) {
  _host = con;
  _columnPrefix = "";
  return open();
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void SQLiteDatabase::disconnect() {
  if (_handle != NULL) {
    sqlite3_close(_handle);
    _handle = NULL;
  }
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::isConnected() const { return _handle != NULL; }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void SQLiteDatabase::start() { execute("begin transaction"); }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void SQLiteDatabase::commit() { execute("commit transaction"); }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void SQLiteDatabase::rollback() { execute("rollback transaction"); }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::execute(const char *command) {
  if (!isConnected() || command == NULL) return false;

  char *errmsg = NULL;
  int result = sqlite3_exec(_handle, command, NULL, NULL, &errmsg);
  if (errmsg != NULL) {
    SEISCOMP_ERROR("sqlite3 execute: %s", errmsg);
    sqlite3_free(errmsg);
  }

  return result == SQLITE_OK;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::beginQuery(const char *query) {
  if (!isConnected() || query == NULL) return false;
  if (_stmt) {
    SEISCOMP_ERROR("beginQuery: nested queries are not supported");
    return false;
  }

  const char *tail;
  int res = sqlite3_prepare(_handle, query, -1, &_stmt, &tail);
  if (res != SQLITE_OK) return false;

  if (_stmt == NULL) return false;

  _columnCount = sqlite3_column_count(_stmt);

  return true;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void SQLiteDatabase::endQuery() {
  if (_stmt) {
    sqlite3_finalize(_stmt);
    _stmt = NULL;
    _columnCount = 0;
  }
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
const char *SQLiteDatabase::defaultValue() const { return "null"; }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
IO::DatabaseInterface::OID SQLiteDatabase::lastInsertId(const char *) {
  sqlite3_int64 id = sqlite3_last_insert_rowid(_handle);
  return id <= 0 ? IO::DatabaseInterface::INVALID_OID : id;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
uint64_t SQLiteDatabase::numberOfAffectedRows() {
  int count = sqlite3_changes(_handle);
  if (count < 0) return (uint64_t)~0;

  return count;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::fetchRow() { return sqlite3_step(_stmt) == SQLITE_ROW; }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int SQLiteDatabase::findColumn(const char *name) {
  for (int i = 0; i < _columnCount; ++i)
    if (!strcmp(sqlite3_column_name(_stmt, i), name)) return i;

  return -1;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int SQLiteDatabase::getRowFieldCount() const { return _columnCount; }
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
const char *SQLiteDatabase::getRowFieldName(int index) {
  return sqlite3_column_name(_stmt, index);
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
const void *SQLiteDatabase::getRowField(int index) {
  return sqlite3_column_blob(_stmt, index);
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
size_t SQLiteDatabase::getRowFieldSize(int index) {
  return sqlite3_column_bytes(_stmt, index);
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bool SQLiteDatabase::escape(std::string &out, const std::string &in) {
  out.resize(in.size() * 2 + 1);
  size_t length = in.length();
  const char *in_buf = in.c_str();
  char *out_buf = &out[0];
  size_t j = 0;

  for (size_t i = 0; i < length && *in_buf; ++length, ++in_buf) {
    switch (*in_buf) {
      case '\'':
        out_buf[j++] = '\'';
        out_buf[j++] = '\'';
        break;
      default:
        out_buf[j++] = *in_buf;
        break;
    }
  }

  out_buf[j] = '\0';
  out.resize(j);
  return true;
}

}  // namespace detail
}  // namespace detect
}  // namespace Seiscomp
