# SCDetect - Computationally efficient earthquake detection

## Content

- [About](#about)
- [Getting started](#getting-started)
  + [General](#general)
  + [Template configuration](#template-configuration)
  + [Waveform data and RecordStream
    configuration](#waveform-data-and-recordstream-configuration)
- [Installation](#compiling-and-installation)
- [Tests](#tests)
- [Issues](#issues)
- [Contributions](#contributions)
- [License](#license)


## About

`scdetect` is a [SeisComP](https://github.com/SeisComP) extension module that
implements both real-time and classical offline earthquake detection based on
waveform cross-correlation, also called matched filtering or template matching.

The module allows both single-stream and multi-stream earthquake detection.

## Getting started

### General

As `scdetect` is a standard SeisComP extension module a list of available
commandline options can be obtained with

```bash
scdetect -h
```

For a general more in-depth introduction on how to use SeisComP modules
including their particular configuration, please refer to the [SeisComP
documentation](https://www.seiscomp.de/doc/index.html).

The subsequent sections are intented to provide an introduction on how to use
and configure `scdetect`. This includes:

1. How to [configure templates](#template-configuration)
2. How is [waveform data
   accessed](#waveform-data-and-recordstream-configuration) including both [waveform
   data caching](#caching-waveform-data) and [template waveform data
   preparation](#prepare-template-waveform-data)

### Template configuration

In order to run `scdetect` a *template configuration* must be provided by means
of using `scdetect`'s `--templates-json path/to/templates.json` CLI flag. The
template configuration is a [JSON](https://www.json.org) configuration file and
contains an array of *detector configuration* JSON objects. An exemplary
multi-stream detector configuration (for the streams `CH.GRIMS..HHZ` and
`CH.HASLI..HHZ`) may look like:

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

That is, a detector configuration defines besides of detector specific
configuration parameters a list of stream configurations and optionally some
[stream configuration defaults](#stream-configuration-defaults).

> **NOTE**: In contrast to other [SeisComP](https://github.com/SeisComP)
> modules (e.g.
> [scautopick](https://docs.gempa.de/seiscomp/current/apps/scautopick.html),
> etc.), `scdetect` is operating strictly stream based. For this reason station
> [bindings](https://www.seiscomp.de/doc/base/concepts/configuration.html#bindings-configuration)
> are not supported.

Note that global defaults may be configured following SeisComP's standard
[module
configuration](https://www.seiscomp.de/doc/base/concepts/configuration.html#module-configuration)
approach.

In order to validate the template configuration, with
[templates.schema.json](src/apps/scdetect/config/templates.schema.json) a
rudimentary [JSON schema](https://json-schema.org/) is provided.

#### Detector configuration parameters

A detector configuration allows setting the following detector specific
configuration parameters:

**General**:

- `"detectorId"`: A string defining the detector identifier. If not defined a
  unique indentifier will be generated, automatically. Since `scdetect`
  implements hierarchical logging specifying the detector identifier may be of
  particular use while debugging.

- `"maximumLatency"`: The maximum data latency in seconds tolerated with
  regards to `NOW`. Data latency is not validated if `scdetect` is run in
  *playback mode*.

- `"originId"`: Required. The origin identifier of the template origin the
  detector is referring to. The origin identifier is used for extracting
  template related data such as template waveforms, etc. Usually, the
  origin identifier corresponds to a *seismic metadata resource identifier*
  (`smi`). The relationship between a detector configuration and an origin is
  one-to-one.

- `"streams"`: Required. An array of stream configuration JSON objects, also
  called a *stream set*.

**Gap interpolation**:

- `"gapInterpolation"`: A boolean value which enables/disables gap
  interpolation.

- `"gapThreshold"`: Threshold in seconds to recognize a gap.

- `"gapTolerance"`: Maximum gap length in seconds to tolerate and to be
  handled.

**Detections and arrivals**:

- `"arrivalOffsetThreshold"`: Maximum arrival offset in seconds to tolerate
  when associating an arrival with an event. Note that the threshold is only
  relevant for a multi-stream detector setup.

- `"createArrivals"`: A boolean value which defines if detections should
  include arrivals with regards to the streams included within the stream set.

- `"createTemplateArrivals"`: A boolean value which defines if detections
  should include *template arrivals*. Template arrivals refer to streams which
  are not part of the detector configuration's stream set, but contain valid
  picks as part of the template origin.

- `"minimumArrivals"`: Defines the minimum number of arrivals which must be
  part of an event to be declared as a detection.

- `"timeCorrection"`: Defines the time correction in seconds for both
  detections and arrivals.

**Trigger facilities**:

- `"triggerDuration"`: Defines the trigger duration in seconds. A negative
  value disables triggering facilities.

- `"triggerOnThreshold"`: Defines the threshold (`[-1, 1]`) to trigger the detector.

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
  undefined a unique indentifier will be generated, automatically. Since `scdetect`
  implements hierarchical logging specifying the template identifier may be of
  particular use while debugging.

- `"initTime"`: The initialization time in seconds for that the stream related
  processor is blind after initialization.

- `"waveformId"`: Required. A string defining the waveform stream identifier of
  the stream to be processed. Usually, this refers to a [FDSN Source
  Identifier](http://docs.fdsn.org/projects/source-identifiers/).

**Template waveform**:

- `"templateWaveformId"`: A string defining an alternative waveform stream
  identifier referring to the stream used for the template waveform creation.
  If not defined, the template waveform is created from the stream specified by
  the `"waveformId"` configuration parameter.

- `"templatePhase"`: Required. A string defining the template phase code used
  for the template waveform creation. Note that for the template phase lookup,
  only the sensor location related part is used of waveform stream identifier
  configured.

- `"templateWaveformStart"`: The template waveform start in seconds with
  regards to the template pick time. A negative value refers to a template
  waveform start *before* the template pick time, while a positive value means
  *after* the pick time.

- `"templateWaveformEnd"`: The template waveform end in seconds with regards to
  the template pick time. A negative value refers to a template waveform start
  *before* the template pick time, while a positive value means *after* the
  pick time.

**Filtering**:

- `"filter"`: A string defining the filter to be applied to the processed
  stream. The filter must be specified following the SeisComP's [filter
  grammar](https://www.seiscomp.de/doc/base/filter-grammar.html) syntax.
  Filtering may be disabled by means of explicitly defining the empty string
  i.e. `""`. By default, the filter associated with the template pick is
  applied.

- `"templateFilter"`: A string defining the filter during the template
  waveform extraction. For further information, please refer to the description
  of the `"filter"` configuration property.

#### Stream configuration defaults

The following stream configuration default parameters may be defined within the
scope of a detector configuration:

- `"filter"`
- `"initTime"`
- `"templateFilter"`
- `"templatePhase"`
- `"templateWaveformStart"`
- `"templateWaveformEnd"`

That is, if not explictly overridden by stream configurations the corresponding
fallback values will be used.

### Waveform data and RecordStream configuration

[SeisComP](https://www.seiscomp.de/) applications access waveform data through
the [RecordStream](https://www.seiscomp.de/doc/base/concepts/recordstream.html)
interface. It is usually configured in
[global.cfg](https://www.seiscomp.de/doc/base/concepts/configuration.html#global-modules-config),
where the user is able to define the backend services in order to access either
real-time and/or historical waveform data. A technical documentation including
exemplary RecordStream configurations can be found
[here](https://www.seiscomp.de/doc/apps/global_recordstream.html#global-recordstream).

Alternatively, the RecordStream can be defined making use of `scdetect`'s `-I [
--record-url ] URI` CLI flag (Note that this is the standard CLI flag used for
all SeisComP modules implementing SeisComP's `StreamApplication` interface.).

In general, with regards to waveform data `scdetect` implements the following
approach:

1. **Initialization**: Download template waveform data from the *archive*
   RecordStream specified.  Cache the raw waveform data (see [Caching waveform
   data](#caching-waveform-data)) and filter the template waveforms according
   to the configuration.

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
rm -rvf ${SEISCOMP_ROOT}/var/cache/scdetect
```

#### Prepare template waveform data

Although, SeisComP allows configuring a [combined
RecordStream](https://www.seiscomp.de/doc/apps/global_recordstream.html#combined),
sometimes it might be useful to fetch template waveform data from a different
RecordStream than the RecordStream providing the data being processed. For this
purpose, `scdetect` provides the `--templates-prepare` CLI flag. With that, an
exemplary processing workflow might look like:

```bash
scdetect \
  --templates-json path/to/templates.json \
  --inventory-db file:///absolute/path/to/inventory.scml \
  --event-db file:///absolute/path/to/catalog.scml \
  --record-url fdsnws://eida-federator.ethz.ch/fdsnws/dataselect/1/query \
  --offline=1 \
  --templates-prepare=1
```

I.e. template waveform data is downloaded from the
[FDSNWS](https://www.seiscomp.de/doc/apps/global_recordstream.html#fdsnws)
RecordStream specified by
`fdsnws://eida-federator.ethz.ch/fdsnws/dataselect/1/query`. After initializing
the module returns.

Next, run the module for processing, but now use the previously cached template
waveform data when loading template waveforms, e.g.

```bash
scdetect \
  --templates-json path/to/templates.json \
  --inventory-db file:///absolute/path/to/inventory.scml \
  --event-db file:///absolute/path/to/catalog.scml \
  --record-url "slink://localhost:18000?timeout=60&retries=5" \
  --offline=1 \
  --ep=detections.scml
```

## Compiling and Installation

Get a copy of
[SeisComP/seiscomp](https://github.com/SeisComP/seiscomp):

```bash
git clone https://github.com/SeisComP/seiscomp.git && cd seiscomp/src/extras/
```

Next, clone `scdetect`:

```bash
git clone https://github.com/damb/scdetect.git
```

For compiling SeisComP (including `scdetect`), please refer to
https://github.com/SeisComP/seiscomp#build.

## Tests

In order to run all SeisComP tests (including those of `scdetect` and possibly
additionally installed third party modules), either execute

```bash
make test
```

in the build directory, or use the
[ctest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) executable from
[cmake](https://cmake.org/) (also within the build directory). E.g. in order to
run only `scdetect` related tests, invoke

```bash
ctest -R "^test_scdetect.*"
```

For additional information, please also refer to SeisComP's [unit testing
guide](https://docs.gempa.de/seiscomp/4/current/base/tests.html).

## Issues

Please report bugs, issues, feature requests, etc on
[GitHub](https://github.com/damb/scdetect/issues).

## Contributions

Contributions are very welcome. Made with :two_hearts:.

## License

Licensed under the the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.en.html).
For more information see the
[LICENSE](https://github.com/damb/scdetect/tree/master/LICENSE) file.
