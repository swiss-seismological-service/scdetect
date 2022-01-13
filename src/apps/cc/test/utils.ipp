#ifndef SCDETECT_APPS_CC_TEST_UTILS_IPP_
#define SCDETECT_APPS_CC_TEST_UTILS_IPP_

#include <vector>

#include "../util/memory.h"

namespace Seiscomp {
namespace detect {
namespace test {

template <Array::DataType TdataType>
GenericRecordPtr makeRecord(size_t count,
                            const ArrayDataTypeTrait_t<TdataType> &value,
                            const Core::Time startTime,
                            double samplingFrequency, std::string chaCode,
                            std::string locCode, std::string staCode,
                            std::string netCode) {
  auto record{util::make_smart<GenericRecord>(
      netCode, staCode, locCode, chaCode, startTime, samplingFrequency, -1,
      TdataType)};

  std::vector<ArrayDataTypeTrait_t<TdataType>> samples(count, value);
  record->setData(samples.size(), samples.data(), TdataType);
  return record;
}

template <Array::DataType TdataType>
GenericRecordPtr makeRecord(
    std::initializer_list<ArrayDataTypeTrait_t<TdataType>> init,
    const Core::Time startTime, double samplingFrequency, std::string chaCode,
    std::string locCode, std::string staCode, std::string netCode) {
  auto record{util::make_smart<GenericRecord>(
      netCode, staCode, locCode, chaCode, startTime, samplingFrequency, -1,
      TdataType)};

  std::vector<ArrayDataTypeTrait_t<TdataType>> samples{init};
  record->setData(samples.size(), samples.data(), TdataType);
  return record;
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_TEST_UTILS_IPP_
