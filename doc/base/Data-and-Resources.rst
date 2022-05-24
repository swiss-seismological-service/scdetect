.. _data-and-resources-label:

Data and Resources
==================

.. _inventory-events-and-configuration-label:

Inventory, events and configuration
-----------------------------------

SeisComP stores and reads certain data (e.g. :external:term:`inventory`
, event parameters, etc.) in and from a database. In order to connect to the
database a *database connection URL* is required. This URL is either configured
in :external:ref:`global.cfg <concepts_modules_config>`
or in :external:ref:`scmaster.cfg <scmaster_configuration>` (i.e. the
configuration file of SeisComP's messaging mediator module,
:external:ref:`scmaster`). In the latter case, it is the
:external:ref:`scmaster` module that passes the database connection
URL to every module connecting to the messaging system (usually at module
startup).

However, when running ``scdetect-cc`` in offline mode (using the CLI option
``--offline``\ ), and the database connection URL is specified in
:external:ref:`scmaster.cfg <scmaster_configuration>`, ``scdetect-cc`` does not
connect to the messaging system and thus, the database connection URL never
reaches ``scdetect-cc``. For this purpose ``scdetect-cc`` provides the standard
SeisComP CLI options:


* ``-d|--database URL``
* ``--inventory-db URI``
* ``--config-db URI``

The non-standard ``--event-db URI`` option allows reading event parameter related
data from either a file or a database specified by ``URI`` (Note that
the ``--event-db URI`` CLI option overrides the ``-d|--database URL`` CLI option.).
With that, :external:term:`inventory` metadata, configuration data and
EventParameters might be read from plain files, making the database
connection fully optional. E.g.

.. code-block:: bash

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

In the example above even
the :ref:`waveform data <waveform-data-and-recordstream-configuration-label>` is read from
a file (i.e.
``data.mseed``\ ). Furthermore, the resulting detections are dumped
:external:term:`SCML` formatted to the ``detections.scml`` file.

For calculating amplitudes ``scdetect-cc`` makes use of
SeisComP's :external:ref:`bindings configuration <global_bindings_config>`
concept (see also :ref:`calculating amplitudes <theory-amplitude-calculation-label>`). In order to
inject :external:ref:`bindings configuration <global_bindings_config>`
the standard SeisComP ``--config-db URI`` CLI flag may be used (as an alternative
to ``-d|--database URL``\ ).

.. _waveform-data-and-recordstream-configuration-label:

Waveform data and RecordStream configuration
--------------------------------------------

`SeisComP <https://www.seiscomp.de/>`_ applications access waveform data through
the :external:term:`RecordStream` interface. It is usually configured in
:external:ref:`global.cfg <global_modules_config>`
, where the user is able to define the backend services in order to access
either real-time and/or historical waveform data. A technical documentation
including exemplary RecordStream configurations can be found
:external:ref:`here <global_recordstream>`.

Alternatively, the RecordStream can be defined making use of ``scdetect-cc``\ '
s ``-I [
--record-url ] URI`` CLI flag (Note that this is the standard CLI flag used for
all SeisComP modules implementing SeisComP's ``StreamApplication`` interface.).

In general, with regard to waveform data ``scdetect-cc`` implements the following
approach:


#. 
   **Initialization**\ : Download template waveform data from the *archive*
   RecordStream specified. Cache the raw waveform data (
   see :ref:`caching-waveform-data-label`) and filter the template
   waveforms according to the configuration.

#. 
   **Processing**\ : Start processing the waveform data from either the
   *real-time* or the *archive* RecordStream configured.

.. _playback-label:

Playback
--------

``scdetect-cc`` may be used to process archived waveform data in the so-called
*playback mode*. A good starting point is
the :external:ref:`SeisComP tutorial on playbacks <tutorials_waveformplayback>`
.

Here, some additional important notes (which may repeat parts of
the :external:ref:`SeisComP tutorial on playbacks <tutorials_waveformplayback>`):


* ``scdetect-cc``\ 's playback mode is enabled with the ``--playback`` CLI flag.
* The maximum record latency (configurable by means of the ``"maximumLatency"``
  detector configuration parameter) is not validated if ``scdetect-cc`` is run in
  playback mode.
* When reading data from a local archive, make sure the records are **sorted by
  end time**. Sorting miniSEED records is easily done
  using :external:ref:`scmssort`.

.. _caching-waveform-data-label:

Caching waveform data
---------------------

Unless the RecordStream points to a local disk storage, downloading waveforms
might require a lot of time. For this reason ``scdetect-cc`` stores raw template
waveform data on disk after downloading them. The cache is located under
``${SEISCOMP_ROOT}/var/cache/scdetect/cc``. If omitting cached waveform data is
desired, make use of ``scdetect-cc``\ 's ``--templates-reload`` CLI flag.

In order to remove cached waveform data, simply invoke

.. code-block:: bash

   rm -rvf ${SEISCOMP_ROOT}/var/cache/scdetect/cc


.. _prepare-template-waveform-data-label:

Prepare template waveform data
------------------------------

Although, SeisComP allows configuring a
:external:ref:`combined RecordStream <rs-combined>`
, sometimes it might be useful to fetch template waveform data from a different
RecordStream than the RecordStream providing the data being processed. For this
purpose, ``scdetect-cc`` provides the ``--templates-prepare`` CLI flag. With that,
an exemplary processing workflow might look like:

.. code-block:: bash

   $ scdetect-cc \
     --templates-json path/to/templates.json \
     --inventory-db file:///absolute/path/to/inventory.scml \
     --event-db file:///absolute/path/to/catalog.scml \
     --record-url fdsnws://eida-federator.ethz.ch/fdsnws/dataselect/1/query \
     --offline \
     --templates-prepare

I.e. template waveform data is downloaded from the
:external:ref:`FDSNWS RecordStream <rs-fdsnws>` specified by
``fdsnws://eida-federator.ethz.ch/fdsnws/dataselect/1/query``. After
initialization the modules exits and returns.

Next, run the module for processing, but now use the previously cached template
waveform data when loading template waveforms, e.g.

.. code-block:: bash

   $ scdetect-cc \
     --templates-json path/to/templates.json \
     --inventory-db file:///absolute/path/to/inventory.scml \
     --event-db file:///absolute/path/to/catalog.scml \
     --record-url "slink://localhost:18000?timeout=60&retries=5" \
     --offline \
     --ep=detections.scml
