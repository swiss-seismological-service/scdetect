.. _template-configuration-label:

Template configuration
======================

In order to run ``scdetect-cc`` a *template configuration* must be provided. That
is, either by means of the ``templatesJSON`` configuration option in one of
``scdetect-cc``\ '
s :external:ref:`module configuration files <concepts_modules_config>`
or by means of using ``scdetect-cc``\ 's ``--templates-json path/to/templates.json``
CLI flag.

The template configuration itself is a `JSON <https://www.json.org>`_
configuration file and contains an array of *detector configuration* JSON
objects (each detector refers to a template origin identified by its
``"originId"``\ ). An exemplary multi-stream detector configuration (for the
streams ``CH.GRIMS..HHZ`` and ``CH.HASLI..HHZ``\ ) may look like:

.. code-block:: json

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

That is, a detector configuration defines besides the detector specific
configuration parameters a list of stream configurations and optionally some
:ref:`stream configuration defaults <stream-configuration-defaults-label>`. Again, the list
of detector configurations is wrapped into a JSON array.

Note that if a configuration parameter is not explicitly defined a module
specific global default is used instead. Global defaults may be configured
following SeisComP's
standard :external:ref:`module configuration <concepts_modules_config>`
approach. That is, by means of either global or module specific configuration
files (\ ``global.cfg``\ , ``scdetect-cc.cfg`` located at corresponding configuration
directory paths).

In order to validate the template configuration, with
`templates.schema.json <src/apps/scdetect/json-schema/templates.schema.json>`_ a
rudimentary `JSON schema <https://json-schema.org/>`_ is provided.

.. _detector-configuration-parameters-label:

Detector configuration parameters
---------------------------------

A detector configuration allows to set the following detector specific
configuration parameters:

**General**\ :


* 
  ``"detectorId"``\ : A string defining the detector identifier. If not defined a
  random unique identifier will be generated, automatically. Declared origins
  will contain a comment with a reference to the detector identifier. Besides,
  the detector identifier is used in the context of the template family
  amplitude-magnitude regression based magnitude estimation.

  Note also that since ``scdetect-cc`` implements hierarchical logging explicitly
  defining the detector identifier may be of particular use while debugging.

* 
  ``"maximumLatency"``\ : The maximum data latency in seconds tolerated with regard
  to ``NOW``. If data arrive later than the value specified it is not used,
  anymore. Note that data latency is not validated if ``scdetect-cc`` is run in
  :ref:`playback mode <playback-label>`.

* 
  ``"methodId"``\ : The origin method identifier which will be added to declared
  origins.

* 
  ``"originId"``\ : Required. The origin identifier of the template origin the
  detector is referring to. The origin identifier is used for extracting
  template related data such as template waveforms, etc. Usually, the origin
  identifier corresponds to a *seismic metadata resource identifier*
  (\ ``smi``\ ). The relationship between a detector configuration and an origin is
  one-to-one.

* 
  ``"streams"``\ : Required. An array of stream configuration JSON objects, also
  called a *stream set*. The stream set describes the streams to be covered by a
  detector. In a single-stream detector configuration the stream set contains
  just a single stream configuration, while a multi-stream detector
  configuration requires multiple stream configurations.

**Gap interpolation**\ :


* 
  ``"gapInterpolation"``\ : A boolean value which enables/disables gap interpolation
  which allows interpolating gaps linearly.

* 
  ``"gapThreshold"``\ : Threshold in seconds to recognize a gap.

* 
  ``"gapTolerance"``\ : Maximum gap length in seconds to tolerate and to be handled.

**Amplitude calculation**\ :

In order to perform a magnitude estimation later on, the corresponding
amplitudes must be computed beforehand. On detector configuration level the
following amplitude related configuration parameters may be provided:


* ``"createAmplitudes"``\ : Boolean value which
  enables/disables :ref:`amplitude calculation <theory-amplitude-calculation-label>` for this
  detector configuration. Note also that amplitudes are calculated only for
  those sensor locations
  where :external:ref:`bindings configuration <global_bindings_config>`
  is supplied.

..

   **NOTE**\ : Magnitudes are computed only for those detectors with amplitude calculation enabled.


**Magnitude estimation**\ :

Magnitude related configuration options which may be defined within a detector
configuration include:


* ``"createMagnitudes"``\ : Boolean value which
  enables/disables :ref:`magnitude estimation <theory-magnitude-estimation-label>` for this
  detector configuration. Magnitudes are only computed for those sensor
  locations where
  both :external:ref:`bindings configuration <global_bindings_config>`
  is available and amplitude calculation is enabled (
  i.e. ``"createAmplitudes": true``\ ).

**Detections and arrivals**\ :


* 
  ``"arrivalOffsetThreshold"``\ : Maximum arrival offset in seconds (i.e. with
  regard to the template arrival) to tolerate when associating an arrival with
  an *association*. Note that the threshold is only relevant for a multi-stream
  detector setup.

* 
  ``"minimumArrivals"``\ : Defines the minimum number of arrivals w.r.t. streams
  within the stream set configured which must be part of an association to
  qualify for a detection.

* 
  ``"mergingStrategy"``\ : Defines the merging strategy applied before linking
  cross-correlation results. Possible configuration options are:


  * ``"greaterEqualTriggerOnThreshold"``\ : cross-correlation results with regard
    to the configured streams must be greater or equal to the configured
    ``"triggerOnThreshold"`` in order to be taken into account for linking.
    Results lower than the ``"triggerOnThreshold"`` are dropped.
  * ``"greaterEqualMergingThreshold"``\ : cross-correlation results with regard to
    the configured streams must be greater or equal to the stream specific
    configured ``"mergingThreshold"`` in order to be taken into account for
    linking. Results lower than the ``"mergingThreshold"`` are dropped.
  * ``"all"``\ : all cross-correlation results with regard to the configured
    streams are taken into account while linking. Trying to merge all incoming
    cross-correlation results is computationally quite expensive.

..

   **NOTE**\ : The configured merging strategy may have a significant performance
   impact in a multi-stream detector setup.



* 
  ``"createArrivals"``\ : A boolean value which defines if detections should
  include *detected arrivals*\ , i.e. arrivals with regard to the streams included
  within the stream set. If enabled, origins will be created with detected
  arrivals being associated, else origins are created not containing any
  reference to detected arrivals. In a multi-stream detector configuration setup
  detections will include only arrivals of those streams which contributed. For
  further details, please refer to ``"createTemplateArrivals"`` and
  ``"minimumArrivals"`` configuration parameters.

* 
  ``"createTemplateArrivals"``\ : A boolean value which defines if detections should
  include so called *template arrivals*. Template arrivals refer to streams
  which are not part of the detector configuration's stream set, but contain
  valid picks as part of the template origin.

* 
  ``"timeCorrection"``\ : Defines the time correction in seconds for both detections
  and arrivals. That is, this allows shifting a detection in time.

**Trigger facilities**\ :

An *association* is considered as a *detected association*\ , also called a
*detection* if it surpasses the value specified by the ``"triggerOnThreshold"``
configuration parameter.

In a multi-stream detector setup, ``scdetect-cc`` uses the *mean* correlation
coefficient of all streams within the stream set. In future, further methods may
be provided in order to compute this *score*.

Besides, ``scdetect-cc`` implements trigger facilities, i.e. a detection may not
be published, immediately, but put *on-hold* for the duration defined by the
value of the ``"triggerDuration"`` configuration parameter. If a *better*
detection arrives within this period, the previous one is not used, anymore.


* 
  ``"triggerDuration"``\ : Defines the trigger duration in seconds. A negative value
  disables triggering facilities.

* 
  ``"triggerOnThreshold"``\ : Defines the threshold (\ ``[-1, 1]``\ ) to trigger the
  detector.

* 
  ``"triggerOffThreshold"``\ : Defines the lower threshold (\ ``[-1, 1]``\ ) to emit a
  detection once the detector is triggered. Note that the configured value is
  only taken into account if trigger facilities are enabled.

..

   **NOTE**\ : With trigger facilities enabled a detection is processed only once
   there is the *next* detection already available. Since processing a detection
   may involve calculating amplitudes the ``processing.waveformBufferSize`` must
   cover the corresponding duration in order to successfully compute amplitudes
   (fetching historical data is currently not implemented, yet).

.. _stream-configuration-parameters-label:

Stream configuration parameters
-------------------------------

A stream set must contain at least a single stream configuration. For a
multi-stream detector setup multiple stream configurations may be provided. A
stream configuration JSON object allows setting the following template specific
configuration parameters:

**General**\ :


* 
  ``"templateId"``\ : A string defining the stream related template identifier. The
  identifier must be unique in the context of a detector configuration. If
  undefined a unique identifier will be generated, automatically.
  Since ``scdetect-cc`` implements hierarchical logging specifying the template
  identifier may be of particular use while debugging.

* 
  ``"waveformId"``\ : Required. A string defining the waveform stream identifier of
  the stream to be processed. Usually, this refers to
  a `FDSN Source Identifier <http://docs.fdsn.org/projects/source-identifiers/>`_.
  Note that the string is parsed and matched against ``NET``\ , ``STA``\ , ``LOC``\ , ``CHA``
  codes.

  By default, the template waveform is created based on the same waveform stream
  identifier. See also ``"templateWaveformId"``.

* 
  ``"mergingThreshold"``\ : Optionally defines a stream configuration specific
  threshold (\ ``[-1, 1]``\ ) which is used exclusively if ``"mergingStrategy"`` is set
  to ``"greaterEqualMergingThreshold"``. If ``"mergingThreshold"`` is not configured
  it is set to the value provided by ``"triggerOnThreshold"``.

**Template waveform**\ :


* ``"templateWaveformId"``\ : A string defining an alternative waveform stream
  identifier referring to the stream used for the template waveform creation. If
  not defined, the template waveform is used as defined by the ``"waveformId"``
  configuration parameter. While for the phase code lookup the *sensor location*
  is used (i.e. the ``CHA``
  component of the waveform stream identifier is neglected) for template
  waveform creation all waveform stream identifier components are taken into
  account.

..

   **NOTE**\ : When specifying a ``"templateWaveformId"`` different from
   ``"waveformId"``\ , ``scdetect-cc`` will not correct potentially differing sensor
   responses.



* 
  ``"templatePhase"``\ : Required. A string defining the template phase code used
  for the template waveform creation. It is the phase code which defines the
  *reference time* for the actual template waveform creation.

  Note that for the template phase lookup, only the sensor location related part
  is used of waveform stream identifier configured.

* 
  ``"templateWaveformStart"``\ : The template waveform start in seconds with regard
  to the template reference time. A negative value refers to a template waveform
  start *before* the template reference time, while a positive value means
  *after* the reference time.

* 
  ``"templateWaveformEnd"``\ : The template waveform end in seconds with regard to
  the template reference time. A negative value refers to a template waveform
  start *before* the template reference time, while a positive value means
  *after* the reference time.

**Filtering and resampling**\ :


* 
  ``"initTime"``\ : The initialization time in seconds for that the stream related
  processor is blind after initialization. Setting this configuration parameter
  allows taking filter related artifacts during initialization into account.

* 
  ``"filter"``\ : A string defining the filter to be applied to the processed
  stream. The filter must be specified following the
  SeisComP's :external:ref:`filter grammar <filter-grammar>`
  syntax. Filtering may be disabled by means of explicitly defining the empty
  string i.e. ``""``. If no module default is configured, by default the filter
  associated with the *template pick* is applied.

* 
  ``"templateFilter"``\ : A string defining the filter during the template waveform
  generation. If ``"templateFilter"`` is undefined, but ``"filter"`` is
  defined ``"templateFilter"`` is automatically configured to the value set
  by ``"filter"``.

  For further information, please refer to the description of the ``"filter"``
  configuration parameter.

* 
  ``"targetSamplingFrequency"``\ : Optionally, defines the target sampling
  frequency. Both the template waveform and the stream to be processed may be
  required to be resampled to the sampling frequency specified. Note that data
  is resampled **before** being filtered.

.. _stream-configuration-defaults-label:

Stream configuration defaults
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following stream configuration default parameters may be defined within the
scope of a detector configuration:


* ``"filter"``
* ``"initTime"``
* ``"mergingThreshold"``
* ``"targetSamplingFrequency"``
* ``"templateFilter"``
* ``"templatePhase"``
* ``"templateWaveformStart"``
* ``"templateWaveformEnd"``

That is, if not explicitly overridden by stream configurations the corresponding
fallback values will be used.

**Example:**

.. code-block:: json

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

In the example above, the stream configuration default ``"templatePhase"`` is used
indicating a default phase code ``"Pg"``. While this stream configuration default
value is used by the stream configuration object identified by the template
identifier ``"template-01"``\ , it is overridden by the stream configuration
identified by ``"template-02"`` (i.e. it uses ``"Sg"`` instead).

Besides, filtering is explicitly disabled for all stream configurations within
the stream set.

