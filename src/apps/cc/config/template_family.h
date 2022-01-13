#ifndef SCDETECT_APPS_SCDETECT_CONFIG_TEMPLATEFAMILY_H_
#define SCDETECT_APPS_SCDETECT_CONFIG_TEMPLATEFAMILY_H_

#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "../util/util.h"
#include "detector.h"

namespace Seiscomp {
namespace detect {
namespace config {

class TemplateFamilyConfig {
 public:
  struct ReferenceConfig;

 private:
  using ReferenceConfigs = std::set<ReferenceConfig>;

 public:
  // Configuration referencing a template family member
  struct ReferenceConfig {
    struct SensorLocationConfig {
      // The phase code
      std::string phase{"Pg"};

      std::string waveformId;
      std::string channelId;

      // Defines the lower limit for magnitude calculation
      boost::optional<double> lowerLimit;
      // Defines the opper limit for magnitude calculation
      boost::optional<double> upperLimit;

      double waveformStart{-2};
      double waveformEnd{2};

      // Compare for order
      bool operator<(const SensorLocationConfig &c) const {
        return waveformId < c.waveformId;
      }
    };

    // The reference configuration's origin identifier
    std::string originId;
    // The optional reference configuration's detector identifier
    boost::optional<std::string> detectorId;

    using SensorLocationConfigs = std::set<SensorLocationConfig>;
    SensorLocationConfigs sensorLocationConfigs;

    ReferenceConfig(
        const boost::property_tree::ptree &pt,
        const std::vector<TemplateConfig> &templateConfigs,
        const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults);

    // Returns whether the configuration actually references a detector
    bool referencesDetector() const;

    // Compare for order
    bool operator<(const ReferenceConfig &c) const {
      return originId < c.originId;
    }

   private:
    using TemplateConfigs = std::vector<TemplateConfig>;
    using TemplateConfigsIdx =
        std::unordered_map<std::string, TemplateConfigs::const_iterator>;
    static void createIndex(const TemplateConfigs &templateConfigs);
    // Returns whether a index has been created for `TemplateConfigs`
    bool indexed() const;

    static TemplateConfigsIdx _templateConfigsIdx;
  };

  TemplateFamilyConfig(
      const boost::property_tree::ptree &pt,
      const std::vector<TemplateConfig> &templateConfigs,
      const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults);

  using size_type = ReferenceConfigs::size_type;
  using value_type = ReferenceConfigs::value_type;
  using iterator = ReferenceConfigs::iterator;
  using const_iterator = ReferenceConfigs::const_iterator;

  size_type size() const noexcept { return _referenceConfigs.size(); }

  iterator begin() { return _referenceConfigs.begin(); }
  iterator end() { return _referenceConfigs.end(); }
  const_iterator begin() const { return _referenceConfigs.begin(); }
  const_iterator end() const { return _referenceConfigs.end(); }
  const_iterator cbegin() const { return _referenceConfigs.cbegin(); }
  const_iterator cend() const { return _referenceConfigs.cend(); }

  // Returns the template family's identifier
  const std::string &id() const;
  // Returns the magnitude type the template family is configured with
  const std::string &magnitudeType() const;

 protected:
  // Loads the template family's reference configurations from `pt` and
  // `templateConfigs`
  void loadReferenceConfigs(
      const boost::property_tree::ptree &pt,
      const std::vector<TemplateConfig> &templateConfigs,
      const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults);

 private:
  void validateMagnitudeType(const std::string &magnitudeType);
  // The template family identifier
  std::string _id{util::createUUID()};

  ReferenceConfigs _referenceConfigs;

  std::string _magnitudeType;
  using AllowedMagnitudeTypes = std::set<std::string>;
  static const AllowedMagnitudeTypes _allowedMagnitudeTypes;
};

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_CONFIG_TEMPLATEFAMILY_H_
