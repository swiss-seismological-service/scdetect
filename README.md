# SCDetect - Computationally efficient earthquake detection

[![continuous-integration](https://github.com/damb/scdetect/actions/workflows/continuous-integration.yml/badge.svg)](https://github.com/damb/scdetect/actions/workflows/continuous-integration.yml)

## Content

- [About](#about)
- [Overview](#overview)
- [Getting started](#getting-started)
    + [General](#general)
    + [Template configuration](#template-configuration)
    + [Amplitude calculation](#amplitude-calculation)
    + [Magnitude estimation](#magnitude-estimation)
    + [Inventory metadata, event metadata and configuration](#inventory-events-and-configuration)
    + [Waveform data and RecordStream configuration](#waveform-data-and-recordstream-configuration)
- [Installation](#compiling-and-installation)
- [Tests](#tests)
- [Issues](#issues)
- [Contributions](#contributions)
- [Alpha Disclaimer](#alpha-disclaimer)
- [License](#license)

## About

SCDetect is a [SeisComP](https://github.com/SeisComP) package which implements
by means of the extension module `scdetect-cc` both real-time and classical
offline earthquake detection based on waveform cross-correlation, also called
matched filtering or template matching. Again, the underlying cross-correlation
algorithm is based on computing
the [Pearson Correlation Coefficient](https://en.wikipedia.org/wiki/Pearson_correlation_coefficient)
.

The module allows both single-stream and multi-stream earthquake detection.

In case the detection parameters exceed the configured thresholds, `scdetect-cc`
declares a new origin.

Besides, magnitudes may be estimated based on
multiple [magnitude estimation](#magnitude-estimation) methods (regression,
amplitude ratios).

## Overview

<p align="center">
  <img border="0" src="doc/diagrams/sc-system-scdetect.svg" width="400"
  title="SCDetect and SeisComP" />
</p>

Above, the modular organization of SeisComP
with [messaging system](https://docs.gempa.de/seiscomp/current/base/concepts/messaging.html)
(mediator),
[RecordStream](https://docs.gempa.de/seiscomp/current/base/concepts/recordstream.html)
interface (waveform server) and
[database](https://docs.gempa.de/seiscomp/current/base/concepts/database.html)
including `scdetect-cc`'s role in the SeisComP overall architecture. In SeisComP
language `scdetect-cc` is a representative for
a [trunk module](https://www.seiscomp.de/doc/base/glossary.html#term-trunk).

From an architectural point of view `scdetect-cc` is positioned somewhere
between
[scautopick](https://docs.gempa.de/seiscomp/current/apps/scautopick.html) and
[scautoloc](https://docs.gempa.de/seiscomp/current/apps/scautoloc.html). That
is, `scdetect-cc` fetches waveform data by means of
the [RecordStream](https://docs.gempa.de/seiscomp/current/base/concepts/recordstream.html)
interface, but it also uses data products (i.e. event parameters) for template
generation. If connected to the messaging system, results (i.e. declared
origins (including station magnitudes), picks and amplitudes) are sent to the
messaging system.

For further information with regard to the SeisComP architecture please refer to
the [SeisComP documentation](https://docs.gempa.de/seiscomp/current/base/overview.html)
.

## Getting started

### General

As `scdetect-cc` is a *standard* SeisComP extension module a list of available
commandline options can be obtained with

```bash
$ scdetect-cc -h
```

For a general more in-depth introduction on how to use SeisComP modules
including their particular configuration, please refer to
the [SeisComP documentation](https://www.seiscomp.de/doc/index.html).

___

The subsequent sections are intended to provide an introduction on how to use
and configure `scdetect-cc`. This includes:

1. How to [configure templates](#template-configuration)
2. How to enable [amplitude calculation](#amplitude-calculation) (required for a
   magnitude estimation later on)
3. How to enable [magnitude estimation](#magnitude-estimation)
4. How to access
   [metadata and configuration](#inventory-events-and-configuration) from the
   database or from plain files
5. How
   [waveform data is accessed](#waveform-data-and-recordstream-configuration)
   including both [template waveform data caching](#caching-waveform-data)
   and [template waveform data preparation](#prepare-template-waveform-data)

### Template configuration

In order to run `scdetect-cc` a *template configuration* must be provided. That
is, either by means of the `templatesJSON` configuration option in one of
`scdetect-cc`'
s [module configuration files](https://www.seiscomp.de/doc/base/concepts/configuration.html#module-configuration)
or by means of using `scdetect-cc`'s `--templates-json path/to/templates.json`
CLI flag.

The template configuration itself is a [JSON](https://www.json.org)
configuration file and contains an array of *detector configuration* JSON
objects (each detector refers to a template origin identified by its
`"originId"`). An exemplary multi-stream detector configuration (for the
streams `CH.GRIMS..HHZ` and `CH.HASLI..HHZ`) may look like:

```json
[
  {
    "detectorId": "detector-01",
    "createArrivals": true,
    "createTemplateArrivals": true,
    "gapInterpolation": true,
    "gapThreshold": 0.1,
    "gapTolerance": 1.5,
    "triggerDuration": -1,
    "triggerOnThreshold": 0.98,
    "triggerOffThreshold": 0,
    "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20201026144442.91156.194937",
    "filter": "",
    "streams": [
      {
        "templateId": "template-01",
        "initTime": 10,
        "templateWaveformStart": -2,
        "templateWaveformEnd": 2,
        "waveformId": "CH.GRIMS..HHZ",
        "templatePhase": "Pg"
      },
      {
        "templateId": "template-02",
        "initTime": 10,
        "templateWaveformStart": -3,
        "templateWaveformEnd": 1,
        "waveformId": "CH.HASLI..HHZ",
        "templatePhase": "Pg"
      }
    ]
  }
]
```

That is, a detector configuration defines besides the detector specific
configuration parameters a list of stream configurations and optionally some
[stream configuration defaults](#stream-configuration-defaults). Again, the list
of detector configurations is wrapped into a JSON array.

Note that if a configuration parameter is not explicitly defined a module
specific global default is used instead. Global defaults may be configured
following SeisComP's
standard [module configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#module-configuration)
approach. That is, by means of either global or module specific configuration
files (`global.cfg`, `scdetect-cc.cfg` located at corresponding configuration
directory paths).

In order to validate the template configuration, with
[templates.schema.json](src/apps/scdetect/json-schema/templates.schema.json) a
rudimentary [JSON schema](https://json-schema.org/) is provided.

#### Detector configuration parameters

A detector configuration allows to set the following detector specific
configuration parameters:

**General**:

- `"detectorId"`: A string defining the detector identifier. If not defined a
  random unique identifier will be generated, automatically. Declared origins
  will contain a comment with a reference to the detector identifier. Besides,
  the detector identifier is used in the context of the template family
  amplitude-magnitude regression based magnitude estimation.

  Note also that since `scdetect-cc` implements hierarchical logging explicitly
  defining the detector identifier may be of particular use while debugging.

- `"maximumLatency"`: The maximum data latency in seconds tolerated with regard
  to `NOW`. If data arrive later than the value specified it is not used,
  anymore. Note that data latency is not validated if `scdetect-cc` is run in
  [*playback mode*](#playback).

- `"methodId"`: The origin method identifier which will be added to declared
  origins.

- `"originId"`: Required. The origin identifier of the template origin the
  detector is referring to. The origin identifier is used for extracting
  template related data such as template waveforms, etc. Usually, the origin
  identifier corresponds to a *seismic metadata resource identifier*
  (`smi`). The relationship between a detector configuration and an origin is
  one-to-one.

- `"streams"`: Required. An array of stream configuration JSON objects, also
  called a *stream set*. The stream set describes the streams to be covered by a
  detector. In a single-stream detector configuration the stream set contains
  just a single stream configuration, while a multi-stream detector
  configuration requires multiple stream configurations.

**Gap interpolation**:

- `"gapInterpolation"`: A boolean value which enables/disables gap interpolation
  which allows interpolating gaps linearly.

- `"gapThreshold"`: Threshold in seconds to recognize a gap.

- `"gapTolerance"`: Maximum gap length in seconds to tolerate and to be handled.

**Amplitude calculation**:

In order to perform a magnitude estimation later on, the corresponding
amplitudes must be computed beforehand. On detector configuration level the
following amplitude related configuration parameters may be provided:

- `"createAmplitudes"`: Boolean value which
  enables/disables [amplitude calculation](#amplitude-calculation) for this
  detector configuration. Note also that amplitudes are calculated only for
  those sensor locations
  where [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
  is supplied.

> **NOTE**: Magnitudes are computed only for those detectors with amplitude calculation enabled.

**Magnitude estimation**:

Magnitude related configuration options which may be defined within a detector
configuration include:

- `"createMagnitudes"`: Boolean value which
  enables/disables [magnitude estimation](#magnitude-estimation) for this
  detector configuration. Magnitudes are only computed for those sensor
  locations where
  both [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
  is available and amplitude calculation is enabled (
  i.e. `"createAmplitudes": true`).

**Detections and arrivals**:

- `"arrivalOffsetThreshold"`: Maximum arrival offset in seconds (i.e. with
  regard to the template arrival) to tolerate when associating an arrival with
  an *association*. Note that the threshold is only relevant for a multi-stream
  detector setup.

- `"minimumArrivals"`: Defines the minimum number of arrivals w.r.t. streams
  within the stream set configured which must be part of an association to
  qualify for a detection.

- `"mergingStrategy"`: Defines the merging strategy applied before linking
  cross-correlation results. Possible configuration options are:
    + `"greaterEqualTriggerOnThreshold"`: cross-correlation results with regard
      to the configured streams must be greater or equal to the configured
      `"triggerOnThreshold"` in order to be taken into account for linking.
      Results lower than the `"triggerOnThreshold"` are dropped.
    + `"greaterEqualMergingThreshold"`: cross-correlation results with regard to
      the configured streams must be greater or equal to the stream specific
      configured `"mergingThreshold"` in order to be taken into account for
      linking. Results lower than the `"mergingThreshold"` are dropped.
    + `"all"`: all cross-correlation results with regard to the configured
      streams are taken into account while linking. Trying to merge all incoming
      cross-correlation results is computationally quite expensive.

> **NOTE**: The configured merging strategy may have a significant performance
> impact in a multi-stream detector setup.

- `"createArrivals"`: A boolean value which defines if detections should
  include *detected arrivals*, i.e. arrivals with regard to the streams included
  within the stream set. If enabled, origins will be created with detected
  arrivals being associated, else origins are created not containing any
  reference to detected arrivals. In a multi-stream detector configuration setup
  detections will include only arrivals of those streams which contributed. For
  further details, please refer to `"createTemplateArrivals"` and
  `"minimumArrivals"` configuration parameters.

- `"createTemplateArrivals"`: A boolean value which defines if detections should
  include so called *template arrivals*. Template arrivals refer to streams
  which are not part of the detector configuration's stream set, but contain
  valid picks as part of the template origin.

- `"timeCorrection"`: Defines the time correction in seconds for both detections
  and arrivals. That is, this allows shifting a detection in time.

**Trigger facilities**:

An *association* is considered as a *detected association*, also called a
*detection* if it surpasses the value specified by the `"triggerOnThreshold"`
configuration parameter.

In a multi-stream detector setup, `scdetect-cc` uses the *mean* correlation
coefficient of all streams within the stream set. In future, further methods may
be provided in order to compute this *score*.

Besides, `scdetect-cc` implements trigger facilities, i.e. a detection may not
be published, immediately, but put *on-hold* for the duration defined by the
value of the `"triggerDuration"` configuration parameter. If a *better*
detection arrives within this period, the previous one is not used, anymore.

- `"triggerDuration"`: Defines the trigger duration in seconds. A negative value
  disables triggering facilities.

- `"triggerOnThreshold"`: Defines the threshold (`[-1, 1]`) to trigger the
  detector.

- `"triggerOffThreshold"`: Defines the lower threshold (`[-1, 1]`) to emit a
  detection once the detector is triggered. Note that the configured value is
  only taken into account if trigger facilities are enabled.

#### Stream configuration parameters

A stream set must contain at least a single stream configuration. For a
multi-stream detector setup multiple stream configurations may be provided. A
stream configuration JSON object allows setting the following template specific
configuration parameters:

**General**:

- `"templateId"`: A string defining the stream related template identifier. The
  identifier must be unique in the context of a detector configuration. If
  undefined a unique identifier will be generated, automatically.
  Since `scdetect-cc` implements hierarchical logging specifying the template
  identifier may be of particular use while debugging.

- `"waveformId"`: Required. A string defining the waveform stream identifier of
  the stream to be processed. Usually, this refers to
  a [FDSN Source Identifier](http://docs.fdsn.org/projects/source-identifiers/).
  Note that the string is parsed and matched against `NET`, `STA`, `LOC`, `CHA`
  codes.

  By default, the template waveform is created based on the same waveform stream
  identifier. See also `"templateWaveformId"`.

- `"mergingThreshold"`: Optionally defines a stream configuration specific
  threshold (`[-1, 1]`) which is used exclusively if `"mergingStrategy"` is set
  to `"greaterEqualMergingThreshold"`. If `"mergingThreshold"` is not configured
  it is set to the value provided by `"triggerOnThreshold"`.

**Template waveform**:

- `"templateWaveformId"`: A string defining an alternative waveform stream
  identifier referring to the stream used for the template waveform creation. If
  not defined, the template waveform is used as defined by the `"waveformId"`
  configuration parameter. While for the phase code lookup the *sensor location*
  is used (i.e. the `CHA`
  component of the waveform stream identifier is neglected) for template
  waveform creation all waveform stream identifier components are taken into
  account.

> **NOTE**: When specifying a `"templateWaveformId"` different from
> `"waveformId"`, `scdetect-cc` will not correct potentially differing sensor
> responses.

- `"templatePhase"`: Required. A string defining the template phase code used
  for the template waveform creation. It is the phase code which defines the
  *reference time* for the actual template waveform creation.

  Note that for the template phase lookup, only the sensor location related part
  is used of waveform stream identifier configured.

- `"templateWaveformStart"`: The template waveform start in seconds with regard
  to the template reference time. A negative value refers to a template waveform
  start *before* the template reference time, while a positive value means
  *after* the reference time.

- `"templateWaveformEnd"`: The template waveform end in seconds with regard to
  the template reference time. A negative value refers to a template waveform
  start *before* the template reference time, while a positive value means
  *after* the reference time.

**Filtering and resampling**:

- `"initTime"`: The initialization time in seconds for that the stream related
  processor is blind after initialization. Setting this configuration parameter
  allows taking filter related artifacts during initialization into account.

- `"filter"`: A string defining the filter to be applied to the processed
  stream. The filter must be specified following the
  SeisComP's [filter grammar](https://www.seiscomp.de/doc/base/filter-grammar.html)
  syntax. Filtering may be disabled by means of explicitly defining the empty
  string i.e. `""`. If no module default is configured, by default the filter
  associated with the *template pick* is applied.

- `"templateFilter"`: A string defining the filter during the template waveform
  generation. If `"templateFilter"` is undefined, but `"filter"` is
  defined `"templateFilter"` is automatically configured to the value set
  by `"filter"`.

  For further information, please refer to the description of the `"filter"`
  configuration parameter.

- `"targetSamplingFrequency"`: Optionally, defines the target sampling
  frequency. Both the template waveform and the stream to be processed may be
  required to be resampled to the sampling frequency specified. Note that data
  is resampled **before** being filtered.

#### Stream configuration defaults

The following stream configuration default parameters may be defined within the
scope of a detector configuration:

- `"filter"`
- `"initTime"`
- `"mergingThreshold"`
- `"targetSamplingFrequency"`
- `"templateFilter"`
- `"templatePhase"`
- `"templateWaveformStart"`
- `"templateWaveformEnd"`

That is, if not explicitly overridden by stream configurations the corresponding
fallback values will be used.

**Example**:

```json
[
  {
    "detectorId": "detector-01",
    "createArrivals": true,
    "createTemplateArrivals": true,
    "gapInterpolation": true,
    "gapThreshold": 0.1,
    "gapTolerance": 1.5,
    "triggerDuration": -1,
    "triggerOnThreshold": 0.98,
    "triggerOffThreshold": 0,
    "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20201026144442.91156.194937",
    "templatePhase": "Pg",
    "filter": "",
    "initTime": 0,
    "streams": [
      {
        "templateId": "template-01",
        "templateWaveformStart": -2,
        "templateWaveformEnd": 2,
        "waveformId": "CH.GRIMS..HHZ"
      },
      {
        "templateId": "template-02",
        "templateWaveformStart": -3,
        "templateWaveformEnd": 1,
        "waveformId": "CH.HASLI..HHZ",
        "templatePhase": "Sg"
      }
    ]
  }
]
```

In the example above, the stream configuration default `"templatePhase"` is used
indicating a default phase code `"Pg"`. While this stream configuration default
value is used by the stream configuration object identified by the template
identifier `"template-01"`, it is overridden by the stream configuration
identified by `"template-02"` (i.e. it uses `"Sg"` instead).

Besides, filtering is explicitly disabled for all stream configurations within
the stream set.

### Amplitude calculation

Computing amplitudes is a prerequisite in order to perform a magnitude
estimation later on. Since multiple magnitude estimation methods are provided,
each magnitude estimation method requires to compute a corresponding amplitude
type. In accordance with the magnitudes methods described in
the [magnitude estimation](#magnitude-estimation) section `scdetect-cc`
implements the following amplitude types to be computed:

- `MRelative`: Amplitude computed as the ratio between the template waveform and
  the detection. The approach is outlined by
  e.g. https://doi.org/10.1038/ngeo697
  and https://doi.org/10.1126/sciadv.1601946 and uses the same instrument
  components as specified by the detector configuration.

- `MLx`: Amplitudes required for the *amplitude-magnitude regression* approach.
  The implementation follows the approach outlined
  in https://doi.org/10.1029/2019JB017468 (section 3.3.3 *Magnitude Estimation*)
  . Amplitudes used for the amplitude-magnitude regression are so called *sensor
  location [RMS (root-mean-square)](https://en.wikipedia.org/wiki/Root_mean_square)
  amplitudes* (i.e. the maximum sample-wise RMS regarding the horizontal
  components for a certain sensor location w.r.t. velocity seismograms).

Amplitudes are calculated once an origin has been declared.

In general, the computation of amplitudes is sensor location dependent. In order
to provide dedicated configuration for different sensor locations `scdetect-cc`
makes use of
SeisComP's [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
concept. Note that amplitudes are calculated only for those sensor locations
with bindings configuration available.

#### Bindings configuration

This section contains a detailed description how to set up the configuration
required for amplitude calculation.

For those users already familiar with
SeisComP's [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
an important note:

> **NOTE**: At the time being, SeisComP's
> [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
> allows configuration to be provided for stations, only. In order to allow
> users to supply configuration on sensor location granularity, `scdetect-cc`
> makes use of so called *sensor location profiles*.

In general, bindings are configured the easiest with SeisComP's configuration
and system management
frontend [scconfig](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
. Alternatively, *key files* may be created manually.

**Creating key files** (`scconfig`):

1. Create an *sensor location profile* for a sensor location. As a bare minimum
   specify at least the `locationCode` (which may be empty) and `channelCode`
   attributes.

   **Tip**: Although not strictly required, it is recommended to use sensible
   profile names. E.g. for a sensor location profile with `locationCode` `00`
   and `channelCode` `HH` naming the profile with e.g. `00_HH` is recommended.

   In both the `locationCode` and the `channelCode` the wildcard
   characters `?` (which matches any single character) and `*` (which matches
   zero to many characters) are allowed.

   **Tip**: Make use of wildcard characters in order to create a *default*
   sensor location profile.

3. Add the profile's name to the list of known `sensorLocationProfiles`. Only
   those profiles are taken into account with a corresponding list entry.
4. (Optional): in case of creating a *binding profile* assign the bindings
   configuration to the corresponding stations.
5. Save the configuration. With that, the bindings configuration is written to
   so called *key files*.

**Dump bindings configurations**:

`scdetect-cc` (and
SeisComP [trunk modules](https://www.seiscomp.de/doc/base/glossary.html#term-trunk)
in general) do not work with key files directly. Therefore, key files need to be
dumped either to a database or into
a [SCML](https://docs.gempa.de/seiscomp/current/base/glossary.html#term-scml)
formatted configuration file (depending on whether a file-based approach is
desired or not). This is done the easiest with
SeisComP's [bindings2cfg](https://www.seiscomp.de/doc/apps/bindings2cfg.html)
utility. E.g.

```bash
$ bindings2cfg --key-dir path/to/key -o config.scml
```

will dump the configuration into
a [SCML](https://docs.gempa.de/seiscomp/current/base/glossary.html#term-scml)
formatted `config.scml` file, while

```bash
$ bindings2cfg --key-dir path/to/key \
  --plugins dbpostgresql -d postgresql://user:password@localhost/seiscomp
```

will dump the configuration to a database named `seiscomp`
([PostgreSQL](https://www.postgresql.org/) backend) on `localhost`. For further
information, please refer to
the [bindings2cfg](https://www.seiscomp.de/doc/apps/bindings2cfg.html)
documentation.

### Magnitude estimation

`scdetect-cc` estimates magnitudes as so called SeisComP *station magnitudes* (
for further details, please refer to
the [scmag documentation](https://docs.gempa.de/seiscomp/current/apps/scmag.html))
. Magnitudes may be estimated for only those sensor locations, the corresponding
magnitude types were computed, previously. In accordance with the amplitude
types described in the [amplitude calculation section](#amplitude-calculation),
the following magnitude types are available:

- `MRelative`: Template-detection ratio based magnitude estimation. Besides, of
  the corresponding amplitudes to be computed, this particular type requires
  station magnitudes to be available
  through [event parameters](#inventory-events-and-configuration).
  (**References**: https://doi.org/10.1038/ngeo697
  , https://doi.org/10.1126/sciadv.1601946)

- `MLx`: Amplitude-magnitude regression based magnitude type. Besides, of the
  corresponding amplitudes to be computed, this particular type requires both
  amplitudes and station magnitudes to be available by means
  of [event parameters](#inventory-events-and-configuration). Moreover, the
  approach is based on so-called *template families* which in fact are groups of
  *related* templates. The
  corresponding [template family configuration](#template-family-configuration)
  must be provided by `scdetect-cc`'
  s `--templates-family-json path/to/templates-family.json` CLI flag.
  (**References**: https://doi.org/10.1029/2019JB017468 (section 3.3.3
  *Magnitude Estimation*))

All magnitude estimation methods listed above are based on the following types
of *template station magnitudes*:

- [`MLh`](https://www.seiscomp.de/doc/apps/global_mlh.html)

- `MLhc`: based on `MLh`, but uses a slightly adjusted relationship (i.e.
  corrected for near-field observations) and allows for station specific
  corrections.

> **NOTE**: Magnitudes of type `MLhc` are preferred over magnitudes of type `MLh`.

Recall, that template station magnitudes must be available through event
parameters (for further details, please refer to the
related [section](#inventory-events-and-configuration) on providing these data
products).

#### Template family configuration

The template family configuration defines a group of templates which are usually
generated by a cluster analysis approach. The members of a template family
define which *reference magnitudes* are part of a regression for a certain group
of templates.

The template family configuration must be provided as a JSON configuration file
which includes a JSON array of *template family configuration objects*. An
exemplary template family configuration file containing the definition for a
single template family may look like:

```json
[
  {
    "id": "e7bc36fc-95e4-45de-a33d-6dcef43ca809",
    "references": [
      {
        "detectorId": "detector-01",
        "streams": [
          {
            "templateWaveformId": "NET.STA00.LOC.HHZ",
            "upperLimit": 2
          }
        ]
      },
      {
        "detectorId": "detector-02",
        "lowerLimit": 0.5,
        "upperLimit": 2.5,
        "streams": [
          {
            "templateWaveformId": "NET.STA01.LOC.HHZ"
          },
          {
            "templateWaveformId": "NET.STA02.LOC.BHE",
            "upperLimit": 2.3
          }
        ]
      },
      {
        "originId": "originPublicId",
        "streams": [
          {
            "templatePhase": "Pg",
            "templateWaveformId": "NET.STA01.LOC.HHZ",
            "templateWaveformStart": -2,
            "templateWaveformEnd": 2
          }
        ]
      }
    ]
  }
]
```

A template family configuration object may have the following attributes:

- `"id"`: The optional template family identifier. Currently, this identifier
  has no special meaning except of the fact that it is used in log messages. If
  no value is specified a unique identifier is created automatically.

- `"references"`: Required. A JSON array of template family members
  configuration objects. Members may reference detectors (defined by
  the [template configuration](#template-configuration)) or third-party
  origins (i.e. origins which are not used for detections but contribute to the
  amplitude-magnitude regression).

**Detector reference configuration**:

The so-called *detector reference* members reference detector configurations
from the [template configuration](#template-configuration). The reference is
established by means of the `"detectorId"` attribute. Detector reference members
imply that

- reference magnitudes are added to the template family. That is, based both on
  the origin identifier (`"originId"`) and the sensor locations (defined by
  corresponding streams (`"streams"`)).

- for the sensor locations referenced new magnitudes will be computed in case of
  detections issued by the corresponding detector.

Detector reference configuration objects allow the following attributes to be
specified:

- `"detectorId"`: Required. Detector identifier used for establishing a
  reference to a detector configuration from
  the [template configuration](#template-configuration).

- `"streams"`: Required. A JSON array of so-called *sensor location
  configuration objects*. Again, in the context of a detector reference, a
  sensor location configuration object defines the attributes:

    - `"templateWaveformId"`: Required. The template waveform identifier
      regarding a detector configuration stream configuration. Required, in
      order to fully establish the relation detector - (template) origin -
      sensor location.

      Usually, this refers to
      a [FDSN Source Identifier](http://docs.fdsn.org/projects/source-identifiers/en/v1.0/)
      . Note that the part relevant for specifying a sensor location is taken
      into account, only.

    - `"lowerLimit"`: The optional lower limit for magnitudes estimated.
      Magnitudes smaller than the limit specified won't be issued.

    - `"upperLimit"`: The optional upper limit for magnitudes estimated.
      Magnitudes greater than the limit specified won't be issued.

**Detector reference configuration defaults**

The following sensor location configuration defaults may be defined within the
scope of a detector reference configuration:

- `"lowerLimit"`
- `"upperLimit"`

That is, if not explicitly overridden within sensor location configurations the
corresponding fallback values will be used, instead.

**Third-party reference configuration**:

The so-called *third-party reference* members reference origins which are not
used for detection, but they contribute to the amplitude-magnitude regression. A
third-party reference configuration object allows the following attributes to be
defined:

- `"originId"`: Required. The origin identifier used to establish a reference to
  an origin in the catalog. Usually, the origin identifier corresponds to a
  *seismic metadata resource identifier* (`smi`).

- `"streams"`: Required. A JSON array of sensor location configuration objects
  (now, in the context of a third-party reference configuration). In the context
  of a third-party reference configuration a sensor location configuration
  allows the following attributes to be specified:

    - `"templateWaveformId"`: Required. The template waveform identifier
      regarding a (template) origin. Required, in order to fully establish the
      relation origin - sensor location.

      Usually, this refers to
      a [FDSN Source Identifier](http://docs.fdsn.org/projects/source-identifiers/en/v1.0/)
      . Note that the part relevant for specifying a sensor location taken is
      into account, only.

    - `"templatePhase"`: Required. A string defining the template phase code
      used for amplitude calculation. It is the phase code which actually
      defines the *reference time* used for waveform extraction.

    - `"templateWaveformStart"`: The template waveform start in seconds with
      regard to the reference time. A negative value refers to a waveform
      start *before* the reference time, while a positive value means *after*
      the reference time.

    - `"templateWaveformEnd"`: The waveform end in seconds with regard to the
      reference time. A negative value refers to a waveform start *before* the
      reference time, while a positive value means *after* the reference time.

### Inventory, events and configuration

SeisComP stores and reads certain data (e.g.
[inventory](https://www.seiscomp.de/doc/base/concepts/inventory.html#concepts-inventory)
, event parameters, etc.) in and from a database. In order to connect to the
database a *database connection URL* is required. This URL is either configured
in
[global.cfg](https://www.seiscomp.de/doc/base/concepts/configuration.html#global-modules-config)
or in `scmaster.cfg` (i.e. the configuration file of SeisComP's messaging
mediator module, [scmaster](https://www.seiscomp.de/doc/apps/scmaster.html)). In
the latter case, it is the `scmaster` module that passes the database connection
URL to every module connecting to the messaging system (usually at module
startup).

However, when running `scdetect-cc` in offline mode (using the CLI option
`--offline`), and the database connection URL is specified in `scmaster.cfg`,
`scdetect-cc` does not connect to the messaging system and thus, the database
connection URL never reaches `scdetect-cc`. For this purpose `scdetect-cc`
provides the standard SeisComP CLI options:

- `-d|--database URL`
- `--inventory-db URI`
- `--config-db URI`

The non-standard `--event-db URI` option allows reading event parameter related
data from either a file or a database specified by `URI` (Note that
the `--event-db URI` CLI option overrides the `-d|--database URL` CLI option.).
With that, inventory metadata, configuration data and event parameters might be
read from plain files, making the database connection fully optional. E.g.

```bash
$ ls -1
catalog.scml
data.mseed
inventory.scml
templates.json
$ scdetect-cc \
  --templates-json templates.json \
  --inventory-db file://$(realpath inventory.scml) \
  --config-db file://$(realpath config.scml) \
  --event-db file://$(realpath catalog.scml) \
  -I file://$(realpath data.mseed) \
  --offline \
  --ep=detections.scml
```

In the example above even
the [waveform data](#waveform-data-and-recordstream-configuration) is read from
a file (i.e.
`data.mseed`). Furthermore, the resulting detections are dumped
[SCML](https://docs.gempa.de/seiscomp/current/base/glossary.html#term-scml)
formatted to the `detections.scml` file.

For calculating amplitudes `scdetect-cc` makes use of
SeisComP's [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
concept (see also [calculating amplitudes](#amplitude-calculation)). In order to
inject [bindings configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
the standard SeisComP `--config-db URI` CLI flag may be used (as an alternative
to `-d|--database URL`).

### Waveform data and RecordStream configuration

[SeisComP](https://www.seiscomp.de/) applications access waveform data through
the [RecordStream](https://www.seiscomp.de/doc/base/concepts/recordstream.html)
interface. It is usually configured in
[global.cfg](https://www.seiscomp.de/doc/base/concepts/configuration.html#global-modules-config)
, where the user is able to define the backend services in order to access
either real-time and/or historical waveform data. A technical documentation
including exemplary RecordStream configurations can be found
[here](https://www.seiscomp.de/doc/apps/global_recordstream.html#global-recordstream)
.

Alternatively, the RecordStream can be defined making use of `scdetect-cc`'
s `-I [
--record-url ] URI` CLI flag (Note that this is the standard CLI flag used for
all SeisComP modules implementing SeisComP's `StreamApplication` interface.).

In general, with regard to waveform data `scdetect-cc` implements the following
approach:

1. **Initialization**: Download template waveform data from the *archive*
   RecordStream specified. Cache the raw waveform data (
   see [Caching waveform data](#caching-waveform-data)) and filter the template
   waveforms according to the configuration.

2. **Processing**: Start processing the waveform data from either the
   *real-time* or the *archive* RecordStream configured.

#### Playback

`scdetect-cc` may be used to process archived waveform data in the so-called
*playback mode*. A good starting point is
the [SeisComP tutorial on playbacks](https://www.seiscomp.de/doc/base/tutorials/waveformplayback.html)
.

Here, some additional important notes (which may repeat parts of
the [SeisComP tutorial on playbacks](https://www.seiscomp.de/doc/base/tutorials/waveformplayback.html)):

- `scdetect-cc`'s playback mode is enabled with the `--playback` CLI flag.
- The maximum record latency (configurable by means of the `"maximumLatency"`
  detector configuration parameter) is not validated if `scdetect-cc` is run in
  playback mode.
- When reading data from a local archive, make sure the records are **sorted by
  end time**. Sorting miniSEED records is easily done
  using [scmssort](https://www.seiscomp.de/doc/apps/scmssort.html).

#### Caching waveform data

Unless the RecordStream points to a local disk storage, downloading waveforms
might require a lot of time. For this reason `scdetect-cc` stores raw template
waveform data on disk after downloading them. The cache is located under
`${SEISCOMP_ROOT}/var/cache/scdetect`. If omitting cached waveform data is
desired, make use of `scdetect-cc`'s `--templates-reload` CLI flag.

In order to remove cached waveform data, simply invoke

```bash
$ rm -rvf ${SEISCOMP_ROOT}/var/cache/scdetect
```

#### Prepare template waveform data

Although, SeisComP allows configuring
a [combined RecordStream](https://www.seiscomp.de/doc/apps/global_recordstream.html#combined)
, sometimes it might be useful to fetch template waveform data from a different
RecordStream than the RecordStream providing the data being processed. For this
purpose, `scdetect-cc` provides the `--templates-prepare` CLI flag. With that,
an exemplary processing workflow might look like:

```bash
$ scdetect-cc \
  --templates-json path/to/templates.json \
  --inventory-db file:///absolute/path/to/inventory.scml \
  --event-db file:///absolute/path/to/catalog.scml \
  --record-url fdsnws://eida-federator.ethz.ch/fdsnws/dataselect/1/query \
  --offline \
  --templates-prepare
```

I.e. template waveform data is downloaded from the
[FDSNWS](https://www.seiscomp.de/doc/apps/global_recordstream.html#fdsnws)
RecordStream specified by
`fdsnws://eida-federator.ethz.ch/fdsnws/dataselect/1/query`. After initializing
the module returns.

Next, run the module for processing, but now use the previously cached template
waveform data when loading template waveforms, e.g.

```bash
$ scdetect-cc \
  --templates-json path/to/templates.json \
  --inventory-db file:///absolute/path/to/inventory.scml \
  --event-db file:///absolute/path/to/catalog.scml \
  --record-url "slink://localhost:18000?timeout=60&retries=5" \
  --offline \
  --ep=detections.scml
```

## Compiling and Installation

Get a copy of
[SeisComP/seiscomp](https://github.com/SeisComP/seiscomp):

```bash
$ git clone https://github.com/SeisComP/seiscomp.git && cd seiscomp/src/extras/
```

Next, clone SCDetect:

```bash
$ git clone https://github.com/damb/scdetect.git
```

**Dependencies**:

Besides of
the [SeisComP core dependencies](https://github.com/SeisComP/seiscomp#prerequisites)
the following packages must be installed to compile SCDetect:

- `libsqlite3-dev` (Debian, Ubuntu), `sqlite-devel` (RedHat, Fedora, CentOS),
  `dev-db/sqlite` (Gentoo)

For compiling SeisComP (including SCDetect), please refer to
https://github.com/SeisComP/seiscomp#build.

## Tests

In order to run all SeisComP tests (including those of `scdetect-cc` and
possibly additionally installed third party modules), either execute

```bash
$ make test
```

in the build directory, or use the
[ctest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) executable from
[cmake](https://cmake.org/) (also within the build directory). E.g. in order to
run only SCDetect related tests, invoke

```bash
$ ctest -R "^test_scdetect.*"
```

For additional information, please also refer to
SeisComP's [unit testing guide](https://docs.gempa.de/seiscomp/current/base/tests.html)
.

## Issues

Please report bugs, issues, feature requests, etc on
[GitHub](https://github.com/damb/scdetect/issues).

## Contributions

Contributions are very welcome. Made with :two_hearts:.

## Alpha Disclaimer

**SCDetect is currently in alpha**. We may change or remove parts of SCDetect's
API when making new releases.

## License

Licensed under the the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.en.html).
For more information see the
[LICENSE](https://github.com/damb/scdetect/tree/master/LICENSE) file.
