.. _real-time-playback-label:

Real-Time Playback
------------------


This example requires a bit more of experience
with `SeisComP <https://www.seiscomp.de/>`_. If you're not familiar with
the `SeisComP <https://www.seiscomp.de/>`_\ , or ``scdetect-cc``\ , yet, we recommend
you to start with the :ref:`offline-playback-label` tutorials, before.

This section will describe how to use ``scdetect-cc`` in order to process data in
a real-time playback fashion.

For further information on playback archived waveform data in general, please
refer to the :external:ref:`SeisComP playback tutorial <tutorials_waveformplayback>`.

Inventory
^^^^^^^^^

We assume that
the :external:term:`inventory` and :external:ref:`bindings configuration <global_bindings_config>`
for the network of interest are configured. For example, if we want to work with
the Swiss Seismological Network (i.e. network code ``CH``\ ) you need to


* 
  download
  the :external:term:`inventory` data
  with, e.g.

  .. code-block:: bash

     wget "http://eida.ethz.ch/fdsnws/station/1/query?net=CH&sta=*&channel=H??&level=response" -O stations.xml

* 
  import
  the :external:term:`inventory` into the SeisComP database

  .. code-block:: bash

     $SEISCOMP_ROOT/bin/seiscomp exec import_inv fdsnxml stations.xml && \
     $SEISCOMP_ROOT/bin/seiscomp exec scinv sync

Bindings configuration
^^^^^^^^^^^^^^^^^^^^^^


* 
  Global bindings are required for each sensor in order to display the waveform
  data
  with :external:ref:`scrttv`.

* 
  Also, create :external:ref:`seedlink`
  bindings configuration and configure the *mseedfifo - mseedfifo_plugin*
  source.

* 
  Finally, create bindings configuration for ``scdetect-cc`` in order to be able
  to successfully compute
  both :ref:`amplitudes <theory-amplitude-calculation-label>` and
  estimate :ref:`magnitudes <theory-magnitude-estimation-label>`.

Template EventParameters
^^^^^^^^^^^^^^^^^^^^^^^^

Make sure that the template EventParameter data is available in your SeisComP
database. Template EventParameter data may be imported to the SeisComP database
with e.g. :external:ref:`scdispatch`.

Waveform data
^^^^^^^^^^^^^

Download both the waveform data the template waveforms are generated from and
the data to be processed.

..

   **NOTE**\ : it is imported that the data to be processed is sorted by end time.


SeisComP configuration
^^^^^^^^^^^^^^^^^^^^^^


* 
  Enable the :external:ref:`seedlink` module
  configuration option :external:ref:`msrtsimul`.

* 
  Set the ``scdetect-cc`` ``templatesJSON`` module configuration parameter if you
  don't want to specify the parameter on the commandline.

* 
  Configure :external:ref:`scevent`
  according to your needs.

Playback
^^^^^^^^


* Restart SeisComP with

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp restart


* Open a few GUIs to monitor :external:ref:`scmaster` and events

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scrttv & \
   $SEISCOMP_ROOT/bin/seiscomp exec scmm & \
   $SEISCOMP_ROOT/bin/seiscomp exec scesv & \
   $SEISCOMP_ROOT/bin/seiscomp exec scolv &


* Finally, start the playback:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec msrtsimul \
     -m historic \
     -v data.mseed & \
   $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
     --playback \
     --start-stop-msg=1 \
     --debug

In practice, to process data with ``scdetect-cc`` in real-time playback fashion
you need:


* a miniseed file sorted by end time
* a ``template.json`` configuration file which contains the information about the
  detectors and the templates, and
* a fully configured SeisComP system.

Optionally, in case you'd like to fetch the earthquake template EventParameter
data from the database they must be imported, beforehand.
