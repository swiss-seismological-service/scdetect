#include <seiscomp/core/array.h>
#include <seiscomp/core/genericrecord.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

template <Array::DataType TDataType>
struct ArrayDataTypeTrait;

template <>
struct ArrayDataTypeTrait<Array::INT> {
  using type = int;
};
template <>
struct ArrayDataTypeTrait<Array::FLOAT> {
  using type = float;
};
template <>
struct ArrayDataTypeTrait<Array::DOUBLE> {
  using type = double;
};

template <Array::DataType TDataType>
using ArrayDataTypeTrait_t = typename ArrayDataTypeTrait<TDataType>::type;

template <Array::DataType TdataType>
GenericRecordPtr makeRecord(size_t count,
                            const ArrayDataTypeTrait_t<TdataType> &value,
                            const Core::Time startTime,
                            double samplingFrequency, std::string chaCode = "C",
                            std::string locCode = "", std::string staCode = "S",
                            std::string netCode = "N");

template <Array::DataType TdataType>
GenericRecordPtr makeRecord(
    std::initializer_list<ArrayDataTypeTrait_t<TdataType>> init,
    const Core::Time startTime, double samplingFrequency,
    std::string chaCode = "C", std::string locCode = "",
    std::string staCode = "S", std::string netCode = "N");

/* ------------------------------------------------------------------------- */
// Fixture implementing CLI parsing facilities
struct CLIParserFixture {
  CLIParserFixture();
  ~CLIParserFixture();

  void setup();
  void teardown();

  static fs::path pathData;
  static bool keepTempdir;
};

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

#include "utils.ipp"
