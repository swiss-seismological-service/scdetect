#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDEPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDEPROCESSOR_H_

#include <seiscomp/datamodel/amplitude.h>

#include <memory>
#include <unordered_map>

#include "factory.h"
#include "magnitude/template_family.h"
#include "processing/processor.h"
#include "template_family.h"

namespace Seiscomp {
namespace detect {

// Abstract interface for magnitude processors
//
// - the implementation is a slim version of
// `Seiscomp::Processing::MagnitudeProcessor`
class MagnitudeProcessor : public processing::Processor {
 public:
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
    // compute
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
    explicit BaseException(const std::string& msg,
                           Status status = Status::kError);
    BaseException();

    // Sets the exception's status
    void setStatus(Status status);
    // Returns the exception's status
    Status status() const;

   private:
    Status _status{Status::kError};
  };

  class Factory {
    using AdaptedFactory = detect::Factory<MagnitudeProcessor, std::string>;

   public:
    using CallbackType = AdaptedFactory::CallbackType;

    static bool registerFactory(const std::string& id, CallbackType callback);

    static bool unregisterFactory(const std::string& id);

    // Emerges the magnitude processor based on `amplitude` or (as a fallback)
    // from `id`
    static std::unique_ptr<MagnitudeProcessor> create(
        const DataModel::Amplitude* amplitude, const std::string& id = "",
        const std::string& processorId = "");

    // Register a template family
    static void registerTemplateFamily(
        std::unique_ptr<TemplateFamily>&& templateFamily);
    // Unregister a template family
    static void unregisterTemplateFamily(const std::string& detectorId,
                                         const std::string& sensorLocationId);

   private:
    using DetectorId = std::string;
    using SensorLocationId = std::string;
    static bool configureTemplateFamily(
        magnitude::TemplateFamilyBased* processor,
        const DataModel::Amplitude* amplitude, const DetectorId& detectorId,
        const SensorLocationId& sensorLocationId,
        const std::string& magnitudeType,
        std::shared_ptr<TemplateFamily>& templateFamily);
    static bool configureLimits(
        std::unique_ptr<MagnitudeProcessor>& ret, const DetectorId& detectorId,
        const SensorLocationId& sensorLocationId,
        const std::shared_ptr<TemplateFamily>& templateFamily);

    using TemplateFamilies = std::unordered_map<
        DetectorId,
        std::unordered_map<SensorLocationId, std::shared_ptr<TemplateFamily>>>;
    static TemplateFamilies& templateFamilies();
  };

  // Returns the type of the magnitude
  const std::string& type() const;

  // Returns the amplitude type of used for computing the magnitude
  //
  // - the default implementation returns `type()`
  virtual std::string amplitudeType() const;

  // Computes the magnitude from `amplitude`
  virtual double compute(const DataModel::Amplitude* amplitude);

  // Finalizes the `magnitude` created by client code
  virtual void finalize(DataModel::StationMagnitude* magnitude) const;

 protected:
  virtual double computeMagnitude(const DataModel::Amplitude* amplitude) = 0;

  // Converts the amplitude unit of `amplitude` into `targetAmplitudeUnit`
  double convertAmplitude(const DataModel::Amplitude* amplitude,
                          const std::string& targetAmplitudeUnit) const;

  void setType(std::string type);

 private:
  // The type of the magnitude
  std::string _type;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDEPROCESSOR_H_
