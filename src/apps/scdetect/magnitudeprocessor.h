#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDEPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDEPROCESSOR_H_

#include <seiscomp/datamodel/amplitude.h>

#include "processor.h"

namespace Seiscomp {
namespace detect {

// Abstract interface for magnitude processors
//
// - the implementation is a slim version of
// `Seiscomp::Processing::MagnitudeProcessor`
class MagnitudeProcessor : public Processor {
 public:
  MagnitudeProcessor(const std::string& id);

  // XXX(damb): From libs/seiscomp/processing/magnitudeprocessor.h
  enum class Status {
    //! Given amplitude is out of range
    kAmplitudeOutOfRange = 1,
    //! Given depth is out of range to continue processing
    kDepthOutOfRange,
    //! Given distance is out of range to continue processing
    kDistanceOutOfRange,
    //! Given period is out of range to continue processing
    kPeriodOutOfRange,
    //! Given amplitude SNR is out of range to continue processing
    kSNROutOfRange,
    //! Either the origin or the sensor location hasn't been set in call to
    // computeMagnitude
    kMetaDataRequired,
    //! The epicentre is out of supported regions
    kEpicenterOutOfRegions,
    //! The receiver is out of supported regions
    kReceiverOutOfRegions,
    //! The entire raypath does not lie entirely in the supported regions
    kRayPathOutOfRegions,
    //! The unit of the input amplitude was not understood
    kInvalidAmplitudeUnit,
    //! The amplitude object was missing
    kMissingAmplitudeObject,
    //! The estimation of the Mw magnitude is not supported
    kMwEstimationNotSupported,
    //! The configuration is not complete
    kIncompleteConfiguration,
    //! Unspecified error
    kError
  };

  // Base class for all MagnitudeProcessor related exceptions
  class BaseException : public Processor::BaseException {
   public:
    BaseException(const std::string& msg, Status status = Status::kError);
    BaseException();

    // Sets the exception's status
    void setStatus(Status status);
    // Returns the exception's status
    Status status() const;

   private:
    Status _status{Status::kError};
  };

  // Returns the type of the magnitude
  const std::string& type() const;

  // Returns the amplitude type of used for computing the magnitude
  //
  // - the default implementation returns `type()`
  std::string amplitudeType() const;

  // Computes the magnitude from `amplitude`
  //
  // - may throw an instance of `BaseException`
  virtual double compute(DataModel::Amplitude* amplitude);

  // Finalizes the `magnitude` created by client code
  virtual void finalizeMagnitude(DataModel::StationMagnitude* magnitude) const;

 protected:
  virtual double computeMagnitude(DataModel::Amplitude* amplitude) = 0;

  // Converts the amplitude unit of `amplitude` into `targetAmplitudeUnit`
  double convertAmplitude(DataModel::Amplitude* amplitude,
                          const std::string& targetAmplitudeUnit) const;

  // The type of the magnitude
  std::string _type;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDEPROCESSOR_H_