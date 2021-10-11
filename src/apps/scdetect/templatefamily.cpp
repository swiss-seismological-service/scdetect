#include "templatefamily.h"

#include <seiscomp/core/exceptions.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include <string>

#include "amplitude/rms.h"
#include "builder.h"
#include "eventstore.h"
#include "log.h"
#include "seiscomp/client/inventory.h"
#include "seiscomp/core/datetime.h"
#include "seiscomp/core/genericrecord.h"
#include "seiscomp/core/timewindow.h"
#include "seiscomp/datamodel/eventparameters.h"
#include "seiscomp/datamodel/sensorlocation.h"
#include "settings.h"
#include "util/three_components.h"
#include "util/util.h"
#include "util/waveform_stream_id.h"
#include "waveformprocessor.h"

namespace Seiscomp {
namespace detect {

const TemplateFamily::Builder::MagnitudeTypeMap
    TemplateFamily::Builder::_magnitudeTypeMap{
        {"Mw", TemplateFamily::MagnitudeType::kMw},
        {"ML", TemplateFamily::MagnitudeType::kML}};

TemplateFamily::Builder::Builder(
    const TemplateFamilyConfig& templateFamilyConfig)
    : _templateFamilyConfig{templateFamilyConfig} {}

TemplateFamily::Builder& TemplateFamily::Builder::setId(
    const boost::optional<std::string>& id) {
  _product->_id = id.value_or(_templateFamilyConfig.id());
  return *this;
}

TemplateFamily::Builder& TemplateFamily::Builder::setStationMagnitudes() {
  for (const auto& referenceConfig : _templateFamilyConfig) {
    const auto origin{EventStore::Instance().getWithChildren<DataModel::Origin>(
        referenceConfig.originId)};
    if (!origin) {
      throw builder::BaseException{"failed to find origin with id: " +
                                   referenceConfig.originId};
    }

    for (const auto& streamConfig : referenceConfig.streamConfigs) {
      const auto& sensorLocationId{streamConfig.waveformId};
      for (size_t i = 0; i < origin->stationMagnitudeCount(); ++i) {
        const auto stationMagnitude{origin->stationMagnitude(i)};
        const auto amplitudeId{stationMagnitude->amplitudeID()};

        const auto amplitude{
            EventStore::Instance().get<DataModel::Amplitude>(amplitudeId)};
        if (!amplitude) {
          continue;
        }

        if (stationMagnitude->type() != _templateFamilyConfig.magnitudeType()) {
          continue;
        }

        DataModel::WaveformStreamID waveformStreamId;
        try {
          waveformStreamId = amplitude->waveformID();
        } catch (Core::ValueException&) {
          try {
            waveformStreamId = stationMagnitude->waveformID();
          } catch (Core::ValueException&) {
            // missing waveform stream identifier
            continue;
          }
        }

        auto magnitudeSensorLocationId{util::WaveformStreamID{
            waveformStreamId.networkCode(), waveformStreamId.stationCode(),
            waveformStreamId.locationCode(), waveformStreamId.channelCode()}
                                           .sensorLocationStreamId()};
        if (magnitudeSensorLocationId != sensorLocationId) {
          continue;
        }

        auto& member{_members[MapKey{origin->publicID(), sensorLocationId}]};
        member.magnitude = stationMagnitude;
      }
    }
  }

  _product->_magnitudeType =
      _magnitudeTypeMap.at(_templateFamilyConfig.magnitudeType());

  return *this;
}

TemplateFamily::Builder& TemplateFamily::Builder::setAmplitudes(
    WaveformHandlerIface* waveformHandler, const binding::Bindings& bindings) {
  for (const auto& referenceConfig : _templateFamilyConfig) {
    const auto origin{EventStore::Instance().getWithChildren<DataModel::Origin>(
        referenceConfig.originId)};
    if (!origin) {
      throw builder::BaseException{"failed to find origin with id: " +
                                   referenceConfig.originId};
    }
    for (const auto& streamConfig : referenceConfig.streamConfigs) {
      // use phase code to determine arrival time (i.e. actually the
      // corresponding pick time is used)
      const auto& arrivalPhase{streamConfig.phase};

      logging::TaggedMessage msg{streamConfig.waveformId};

      DataModel::PickCPtr pick;
      DataModel::SensorLocationCPtr sensorLocation;
      // determine the pick
      for (size_t i = 0; i < origin->arrivalCount(); ++i) {
        const auto arrival{origin->arrival(i)};
        if (arrival->phase() != arrivalPhase) {
          continue;
        }
        auto p{EventStore::Instance().get<DataModel::Pick>(arrival->pickID())};
        if (!p) {
          continue;
        }
        // validate both arrival and pick
        try {
          if (p->evaluationMode() != DataModel::MANUAL &&
              (arrival->weight() == 0 || !arrival->timeUsed())) {
            continue;
          }
        } catch (Core::ValueException& e) {
          continue;
        }

        util::WaveformStreamID waveformId{streamConfig.waveformId};
        // validate the pick's sensor location
        auto templateWaveformSensorLocation{
            Client::Inventory::Instance()->getSensorLocation(
                waveformId.netCode(), waveformId.staCode(),
                waveformId.locCode(), p->time().value())};
        if (!templateWaveformSensorLocation) {
          msg.setText("failed to find sensor location in inventory for time: " +
                      p->time().value().iso());
          throw builder::NoSensorLocation{logging::to_string(msg)};
        }
        auto pickWaveformId{p->waveformID()};
        auto pickWaveformSensorLocation{
            Client::Inventory::Instance()->getSensorLocation(
                pickWaveformId.networkCode(), pickWaveformId.stationCode(),
                pickWaveformId.locationCode(), p->time().value())};
        if (!pickWaveformSensorLocation ||
            *templateWaveformSensorLocation != *pickWaveformSensorLocation) {
          continue;
        }

        pick = p;
        sensorLocation = templateWaveformSensorLocation;
        break;
      }

      if (!pick) {
        msg.setText("failed to load pick from event parameters");
        throw builder::NoPick{logging::to_string(msg)};
      }

      const auto arrivalTime{pick->time().value()};

      util::WaveformStreamID waveformId{streamConfig.waveformId};
      amplitude::RMSAmplitude rmsAmplitudeProcessor(
          _templateFamilyConfig.id() + settings::kProcessorIdSep +
          referenceConfig.originId + settings::kProcessorIdSep +
          streamConfig.waveformId);

      Core::TimeWindow tw{
          arrivalTime + Core::TimeSpan{streamConfig.waveformStart},
          arrivalTime + Core::TimeSpan{streamConfig.waveformEnd}};
      rmsAmplitudeProcessor.setTimeWindow(tw);

      rmsAmplitudeProcessor.setEnvironment(origin, sensorLocation, {pick});

      rmsAmplitudeProcessor.setResultCallback(
          [this](const WaveformProcessor* proc, const Record* rec,
                 const WaveformProcessor::ResultCPtr& res) {
            storeAmplitude(dynamic_cast<const AmplitudeProcessor*>(proc), rec,
                           boost::dynamic_pointer_cast<
                               const AmplitudeProcessor::Amplitude>(res));
          });

      try {
        util::ThreeComponents threeComponents{
            Client::Inventory::Instance(), waveformId.netCode(),
            waveformId.staCode(),          waveformId.locCode(),
            waveformId.chaCode(),          arrivalTime};

        auto& sensorLocationBindings{
            bindings.at(waveformId.netCode(), waveformId.staCode(),
                        waveformId.locCode(), waveformId.chaCode())};
        auto& amplitudeProcessingConfig{
            sensorLocationBindings.amplitudeProcessingConfig};

        // configure filter
        {
          std::string err;
          auto filter{WaveformProcessor::Filter::Create(
              amplitudeProcessingConfig.filter, &err)};

          if (!filter) {
            msg.setText("failed to initialize filter: " +
                        amplitudeProcessingConfig.filter);
            throw builder::BaseException{logging::to_string(msg)};
          }
          rmsAmplitudeProcessor.setFilter(filter,
                                          amplitudeProcessingConfig.initTime);
        }

        rmsAmplitudeProcessor.setSaturationThreshold(
            amplitudeProcessingConfig.saturationThreshold);

        // register components
        for (auto s : threeComponents) {
          Processing::Stream stream;
          stream.init(s);

          const AmplitudeProcessor::DeconvolutionConfig deconvolutionConfig{
              sensorLocationBindings.at(s->code()).deconvolutionConfig};
          rmsAmplitudeProcessor.add(waveformId.netCode(), waveformId.staCode(),
                                    waveformId.locCode(), stream,
                                    deconvolutionConfig);
        }

        WaveformHandlerIface::ProcessingConfig processingConfig;
        // let the amplitude processor decide how to process the waveform
        processingConfig.demean = false;
        // load waveforms for all components and feed the data to the processor
        for (auto c : threeComponents) {
          GenericRecordCPtr record;
          record = waveformHandler->get(
              threeComponents.netCode(), threeComponents.staCode(),
              threeComponents.locCode(), c->code(),
              rmsAmplitudeProcessor.safetyTimeWindow(), processingConfig);
          rmsAmplitudeProcessor.feed(record.get());
        }
      } catch (std::out_of_range&) {
        msg.setText("failed to load bindings configuration");
        throw builder::NoBindings{logging::to_string(msg)};
      } catch (WaveformProcessor::BaseException& e) {
        msg.setText("failed to load data");
        throw builder::BaseException{logging::to_string(msg)};
      } catch (Exception& e) {
        msg.setText("failed to load streams from inventory for time: " +
                    arrivalTime.iso());
        throw builder::NoStream{logging::to_string(msg)};
      }

      if (rmsAmplitudeProcessor.status() !=
          WaveformProcessor::Status::kFinished) {
        msg.setText(
            "failed compute rms amplitude: status=" +
            std::to_string(util::asInteger(rmsAmplitudeProcessor.status())));
        throw builder::BaseException{logging::to_string(msg)};
      }
    }
  }
  return *this;
}

void TemplateFamily::Builder::finalize() {
  for (auto& memberPair : _members) {
    auto& member{memberPair.second};
    if (member.amplitude && member.magnitude) {
      _product->_members.push_back(member);
    }
  }
}

void TemplateFamily::Builder::storeAmplitude(
    const AmplitudeProcessor* processor, const Record* record,
    const AmplitudeProcessor::AmplitudeCPtr& amplitude) {
  Core::TimeWindow tw{
      amplitude->time.reference - Core::TimeSpan{amplitude->time.begin},
      amplitude->time.reference + Core::TimeSpan{amplitude->time.end}};
  SCDETECT_LOG_DEBUG_TAGGED(
      processor->id(),
      "Creating amplitude (value=%f, starttime=%s, endtime=%s) ...",
      amplitude->value.value, tw.startTime().iso().c_str(),
      tw.endTime().iso().c_str());

  DataModel::AmplitudePtr amp{DataModel::Amplitude::Create()};
  if (!amp) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        processor, "Internal error: duplicate amplitude identifier");
    return;
  }

  amp->setType(processor->type());
  amp->setUnit(processor->unit());
  if (amplitude->waveformStreamIds) {
    if (amplitude->waveformStreamIds.value().size() == 1) {
      const util::WaveformStreamID waveformStreamId{
          amplitude->waveformStreamIds.value()[0]};
      amp->setWaveformID(DataModel::WaveformStreamID{
          waveformStreamId.netCode(), waveformStreamId.staCode(),
          waveformStreamId.locCode(), waveformStreamId.chaCode(), ""});
    }
  }

  amp->setAmplitude(DataModel::RealQuantity{
      amplitude->value.value, boost::none, amplitude->value.lowerUncertainty,
      amplitude->value.upperUncertainty, boost::none});
  amp->setTimeWindow(DataModel::TimeWindow{
      amplitude->time.reference, amplitude->time.begin, amplitude->time.end});

  amp->setSnr(amplitude->snr);
  if (amplitude->dominantPeriod) {
    amp->setPeriod(DataModel::RealQuantity{*amplitude->dominantPeriod});
  }
  processor->finalize(amp.get());

  auto sensorLocationId{
      util::WaveformStreamID{record->networkCode(), record->stationCode(),
                             record->locationCode(), record->channelCode()}
          .sensorLocationStreamId()};
  auto& member{_members[MapKey{processor->environment().hypocenter->publicID(),
                               sensorLocationId}]};
  member.amplitude = amp;
}

/* ------------------------------------------------------------------------- */
TemplateFamily::TemplateFamily() {}

TemplateFamily::Builder TemplateFamily::Create(
    const TemplateFamilyConfig& templateFamilyConfig) {
  return Builder(templateFamilyConfig);
}

const std::string& TemplateFamily::id() const { return _id; }

const TemplateFamily::MagnitudeType& TemplateFamily::magnitudeType() const {
  return _magnitudeType;
}

}  // namespace detect
}  // namespace Seiscomp
