# SCDetect - Computationally efficient earthquake detection

[![continuous-integration](https://github.com/damb/scdetect/actions/workflows/continuous-integration.yml/badge.svg)](https://github.com/damb/scdetect/actions/workflows/continuous-integration.yml)

## Content

- [About](#about)
- [Overview](#overview)
- [Getting started](#getting-started)
    + [General](#general)
    + [Template configuration](#template-configuration)
    + [Inventory metadata, event metadata and configuration](#inventory-events-and-configuration)
    + [Waveform data and RecordStream configuration](#waveform-data-and-recordstream-configuration)
- [Installation](#compiling-and-installation)
- [Tests](#tests)
- [Issues](#issues)
- [Contributions](#contributions)
- [Alpha Disclaimer](#alpha-disclaimer)
- [License](#license)

## About

`scdetect` is a [SeisComP](https://github.com/SeisComP) extension module that
implements both real-time and classical offline earthquake detection based on
waveform cross-correlation, also called matched filtering or template matching.

The module allows both single-stream and multi-stream earthquake detection.

In case the detection parameters exceed the configured thresholds, `scdetect`
declares a new origin.

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
including `scdetect`'s role in the SeisComP overall architecture. In SeisComP
language `scdetect` is a representative for
a [trunk module](https://www.seiscomp.de/doc/base/glossary.html#term-trunk).

From an architectural point of view `scdetect` is positioned somewhere between
[scautopick](https://docs.gempa.de/seiscomp/current/apps/scautopick.html) and
[scautoloc](https://docs.gempa.de/seiscomp/current/apps/scautoloc.html). That
is, `scdetect` fetches waveform data by means of the RecordStream interface, but
it also uses data products for template generation.

For further information with regard to the SeisComP architecture please refer to
the [SeisComP documentation](https://docs.gempa.de/seiscomp/current/base/overview.html)
.

## Getting started

### General

As `scdetect` is a standard SeisComP extension module a list of available
commandline options can be obtained with

```bash
$ scdetect -h
```

For a general more in-depth introduction on how to use SeisComP modules
including their particular configuration, please refer to
the [SeisComP documentation](https://www.seiscomp.de/doc/index.html).

The subsequent sections are intended to provide an introduction on how to use
and configure `scdetect`. This includes:

1. How to [configure templates](#template-configuration)
2. How to access
   [metadata and configuration](#inventory-events-and-configuration) from the
   database or from plain files
3. How
   [waveform data is accessed](#waveform-data-and-recordstream-configuration)
   including both [template waveform data caching](#caching-waveform-data)
   and [template waveform data preparation](#prepare-template-waveform-data)

### Template configuration

In order to run `scdetect` a *template configuration* must be provided. That is,
either by means of the `templatesJSON` configuration option in one of
`scdetect`'
s [module configuration files](https://www.seiscomp.de/doc/base/concepts/configuration.html#module-configuration)
or by means of using `scdetect`'s `--templates-json path/to/templates.json` CLI
flag.

The template configuration itself is a [JSON](https://www.json.org)
configuration file and contains an array of *detector configuration* JSON
objects (each detector refers to a template origin identified by its
`"originId"`). An exemplary multi-stream detector configuration (for the
streams `CH.GRIMS..HHZ` and `CH.HASLI..HHZ`) may look like:

```json
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
  "templateFilter": "",
  "streams": [
    {
      "templateId": "template-01",
      "initTime": 10,
      "templateWaveformStart": -2,
      "templateWaveformEnd": 2,
      "waveformId": "CH.GRIMS..HHZ",
      "templateWaveformId": "CH.GRIMS..HHZ",
      "templatePhase": "Pg"
    },
    {
      "templateId": "template-02",
      "initTime": 10,
      "templateWaveformStart": -3,
      "templateWaveformEnd": 1,
      "waveformId": "CH.HASLI..HHZ",
      "templateWaveformId": "CH.HASLI..HHZ",
      "templatePhase": "Pg"
    }
  ]
}
```

That is, a detector configuration defines besides the detector specific
configuration parameters a list of stream configurations and optionally some
[stream configuration defaults](#stream-configuration-defaults).

> **NOTE**: `scdetect` is operating strictly stream based. For this reason station
> [bindings](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
> are not supported (in contrast to other
> [SeisComP](https://github.com/SeisComP) modules such as e.g.
> [scautopick](https://docs.gempa.de/seiscomp/current/apps/scautopick.html),
> etc.)

Note that if a configuration parameter is not explicitly defined a module
specific global default is used instead. Global defaults may be configured
following SeisComP's
standard [module configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#module-configuration)
approach. That is, by means of either global or module specific configuration
files (`global.cfg`, `scdetect.cfg` located at corresponding configuration
directory paths).

In order to validate the template configuration, with
[templates.schema.json](src/apps/scdetect/config/templates.schema.json) a
rudimentary [JSON schema](https://json-schema.org/) is provided.

#### Detector configuration parameters

A detector configuration allows to set the following detector specific
configuration parameters:

**General**:

- `"detectorId"`: A string defining the detector identifier. If not defined a
  random unique identifier will be generated, automatically. Declared origins
  will contain a comment with a reference to the detector identifier. Note also
  that since `scdetect` implements hierarchical logging explicitly defining the
  detector identifier may be of particular use while debugging.

- `"maximumLatency"`: The maximum data latency in seconds tolerated with regard
  to `NOW`. If data arrive later than the value specified it is not used,
  anymore. Note that data latency is not validated if `scdetect` is run in
  *playback mode*.

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

In a multi-stream detector setup, `scdetect` uses the *mean* correlation
coefficient of all streams within the stream set. In future, further methods may
be provided in order to compute this *score*.

Besides, `scdetect` implements trigger facilities, i.e. a detection may not be
published, immediately, but put *on-hold* for the duration defined by the value
of the `"triggerDuration"` configuration parameter. If a *better* detection
arrives within this period, the previous one is not used, anymore.

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

- `"templateId"`: A string defining the stream related template identifier. If
  undefined a unique identifier will be generated, automatically.
  Since `scdetect`
  implements hierarchical logging specifying the template identifier may be of
  particular use while debugging.

- `"waveformId"`: Required. A string defining the waveform stream identifier of
  the stream to be processed. Usually, this refers to
  a [FDSN Source Identifier](http://docs.fdsn.org/projects/source-identifiers/).
  Note that the string is parsed and matched against `NET`, `STA`, `LOC`, `CHA`
  codes.

- `"mergingThreshold"`: Optionally defines a stream configuration specific
  threshold (`[-1, 1]`) which is used exclusively if `"mergingStrategy"` is set
  to `"greaterEqualMergingThreshold"`. If `"mergingThreshold"` is not configured
  it is set to the value provided by `"triggerOnThreshold"`.

**Template waveform**:

- `"templateWaveformId"`: A string defining an alternative waveform stream
  identifier referring to the stream used for the template waveform creation. If
  not defined, the template waveform is created from the stream specified by
  the `"waveformId"` configuration parameter. While for the phase code lookup
  the *sensor location* is used (i.e. the `CHA`
  component of the waveform stream identifier is neglected) for template
  waveform creation all waveform stream identifier components are taken into
  account.

- `"templatePhase"`: Required. A string defining the template phase code used
  for the template waveform creation. Note that for the template phase lookup,
  only the sensor location related part is used of waveform stream identifier
  configured.

- `"templateWaveformStart"`: The template waveform start in seconds with regard
  to the template pick time. A negative value refers to a template waveform
  start *before* the template pick time, while a positive value means
  *after* the pick time.

- `"templateWaveformEnd"`: The template waveform end in seconds with regard to
  the template pick time. A negative value refers to a template waveform start
  *before* the template pick time, while a positive value means *after* the pick
  time.

**Filtering and resampling**:

- `"initTime"`: The initialization time in seconds for that the stream related
  processor is blind after initialization. Setting this configuration parameter
  allows taking filter related artifacts during initialization into account.

- `"filter"`: A string defining the filter to be applied to the processed
  stream. The filter must be specified following the
  SeisComP's [filter grammar](https://www.seiscomp.de/doc/base/filter-grammar.html)
  syntax. Filtering may be disabled by means of explicitly defining the empty
  string i.e. `""`. By default, the filter associated with the template pick is
  applied.

- `"templateFilter"`: A string defining the filter during the template waveform
  generation. For further information, please refer to the description of
  the `"filter"` configuration property.

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
  "templateFilter": "",
  "initTime": 0,
  "streams": [
    {
      "templateId": "template-01",
      "templateWaveformStart": -2,
      "templateWaveformEnd": 2,
      "waveformId": "CH.GRIMS..HHZ",
      "templateWaveformId": "CH.GRIMS..HHZ"
    },
    {
      "templateId": "template-02",
      "templateWaveformStart": -3,
      "templateWaveformEnd": 1,
      "waveformId": "CH.HASLI..HHZ",
      "templateWaveformId": "CH.HASLI..HHZ",
      "templatePhase": "Sg"
    }
  ]
}
```

In the example above, the stream configuration default `"templatePhase"` is used
indicating a default phase code `"Pg"`. While this stream configuration default
value is used by the stream configuration object identified by the template
identifier `"template-01"`, it is overridden by the stream configuration
identified by `"template-02"` (i.e. it uses `"Sg"` instead).

Besides, filtering is explicitly disabled for all stream configurations within
the stream set.

### Inventory, events and configuration

SeisComP stores and reads certain data (e.g.
[inventory](https://www.seiscomp.de/doc/base/concepts/inventory.html#concepts-inventory)
, eventparameters, etc.) in and from a database. In order to connect to the
database a *database connection URL* is required. This URL is either configured
in
[global.cfg](https://www.seiscomp.de/doc/base/concepts/configuration.html#global-modules-config)
or in `scmaster.cfg` (i.e. the configuration file of SeisComP's messaging
mediator module, [scmaster](https://www.seiscomp.de/doc/apps/scmaster.html)). In
the latter case, it is the `scmaster` module that passes the database connection
URL to every module connecting to the messaging system (usually at module
startup).

However, when running `scdetect` in offline mode (using the CLI option
`--offline`), and the database connection URL is specified in `scmaster.cfg`,
`scdetect` does not connect to the messaging system and thus, the database
connection URL never reaches `scdetect`. For this purpose `scdetect` provides
the standard SeisComP CLI options:

- `-d|--database URL`
- `--inventory-db URI`
- `--config-db URI`

The non-standard `--event-db URI` option allows reading eventparameter related
data from either a file or a database specified by `URI` (Note that
the `--event-db URI` CLI option overrides the `-d|--database URL` CLI option.).
With that, both inventory metadata and eventparameters might be read from plain
files, making the database connection fully optional. E.g.

```bash
$ ls -1
catalog.scml
data.mseed
inventory.scml
templates.json
$ scdetect \
  --templates-json templates.json \
  --inventory-db file://$(realpath inventory.scml) \
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

Note that as the `--config-db URI` CLI option is a standard SeisComP module CLI
option it is mentioned for completeness, only. Since `scdetect` does not
support [station bindings](https://www.seiscomp.de/doc/base/concepts/configuration.html#module-and-bindings-configuration)
, anyway, this CLI can be neglected.

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

Alternatively, the RecordStream can be defined making use of `scdetect`'s `-I [
--record-url ] URI` CLI flag (Note that this is the standard CLI flag used for
all SeisComP modules implementing SeisComP's `StreamApplication` interface.).

In general, with regard to waveform data `scdetect` implements the following
approach:

1. **Initialization**: Download template waveform data from the *archive*
   RecordStream specified. Cache the raw waveform data (
   see [Caching waveform data](#caching-waveform-data)) and filter the template
   waveforms according to the configuration.

2. **Processing**: Start processing the waveform data from either the
   *real-time* or the *archive* RecordStream configured.

#### Caching waveform data

Unless the RecordStream points to a local disk storage, downloading waveforms
might require a lot of time. For this reason `scdetect` stores raw template
waveform data on disk after downloading them. The cache is located under
`${SEISCOMP_ROOT}/var/cache/scdetect`. If omitting cached waveform data is
desired, make use of `scdetect`'s `--templates-reload` CLI flag.

In order to remove cached waveform data, simply invoke

```bash
$ rm -rvf ${SEISCOMP_ROOT}/var/cache/scdetect
```

#### Prepare template waveform data

Although, SeisComP allows configuring
a [combined RecordStream](https://www.seiscomp.de/doc/apps/global_recordstream.html#combined)
, sometimes it might be useful to fetch template waveform data from a different
RecordStream than the RecordStream providing the data being processed. For this
purpose, `scdetect` provides the `--templates-prepare` CLI flag. With that, an
exemplary processing workflow might look like:

```bash
$ scdetect \
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
$ scdetect \
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

Next, clone `scdetect`:

```bash
$ git clone https://github.com/damb/scdetect.git
```

**Dependencies**:

Besides of
the [SeisComP core dependencies](https://github.com/SeisComP/seiscomp#prerequisites)
the following packages must be installed to compile `scdetect`:

- `libsqlite3-dev` (Debian, Ubuntu), `sqlite-devel` (RedHat, Fedora, CentOS),
  `dev-db/sqlite` (Gentoo)

For compiling SeisComP (including `scdetect`), please refer to
https://github.com/SeisComP/seiscomp#build.

## Tests

In order to run all SeisComP tests (including those of `scdetect` and possibly
additionally installed third party modules), either execute

```bash
$ make test
```

in the build directory, or use the
[ctest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) executable from
[cmake](https://cmake.org/) (also within the build directory). E.g. in order to
run only `scdetect` related tests, invoke

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
