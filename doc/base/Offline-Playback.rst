.. _offline-playback-label:

Offline Playback
----------------


*Offline Playback*\ s allow you to process historical data without the need to run
a :external:ref:`seedlink` server. Therefore, the processing is going to be
mostly file based. This approach is ideal if you quickly want to process a
chunk of data, dump the results to a file which may be used for further
processing.

The following examples are provided:


* :ref:`Single detector - Single station - Multi-channel detection <ex-01-label>`
  describes in detail how to reproduce the configuration and results from
  the :ref:`getting-started-label` section.
* :ref:`Single detector - Multi-station - Multi-channel detection <ex-02-label>`
  describes how to configure a single template using streams from multiple
  stations.
* :ref:`Multi-detector - Multi-station - Multi-channel detection <ex-03-label>`
  describes how to configure multiple templates using streams from multiple
  stations.

.. _ex-01-label:

Example: Single detector - Single station - Multi-channel detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tutorial will guide you to reproduce the configuration and data from
the :ref:`getting-started-label` section.

Setup
~~~~~

In order keep things clean and tidy create your project directory and change
into it:

.. code-block:: bash

   mkdir -p $HOME/tmp/scdetect-cc/ex-01 && cd  $HOME/tmp/scdetect-cc/ex-01

Template Configuration
~~~~~~~~~~~~~~~~~~~~~~

``scdetect-cc`` relies on a so-called template configuration (JSON formatted).
Although, in general, multiple templates may be configured, within this example
only a single template is set up.

.. code-block:: json

   [
     {
       "detectorId": "detector-01",
       "createArrivals": true,
       "createTemplateArrivals": true,
       "createAmplitudes": true,
       "createMagnitudes": true,
       "gapInterpolation": true,
       "gapThreshold": 0.1,
       "gapTolerance": 1.5,
       "triggerDuration": 0.5,
       "triggerOnThreshold": 0.7,
       "triggerOffThreshold": 0.0,
       "arrivalOffsetThreshold": 0.2,
       "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990",
       "filter": "BW_BP(2,1.5,15)",
       "templateFilter": "BW_BP(2,1.5,15)",
       "streams": [
         {
           "templateId": "template-01",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SENIN..HHZ",
           "templateWaveformId": "CH.SENIN..HHZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-02",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHN",
           "templateWaveformId": "CH.SENIN..HHN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-03",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHE",
           "templateWaveformId": "CH.SENIN..HHE",
           "templatePhase": "Sg"
         }
       ]
     }
   ]

Above a template configuration based on the template event identified by
the ``originId`` ``smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990``
covering the streams ``HHZ``\ , ``HHN``\ , and ``HHE`` of the sensor location ``CH.SENIN..``
. For the vertical instrument component (channel code ``HHZ``\ ) the template
waveform is referring to the ``templatePhase`` ``Pg``\ , while for the horizontal
instrument components (channel codes ``HHN`` and ``HHE``\ ) are referring to
the ``templatePhase`` ``Sg``.

For a more detailed description of the available configuration options please
refer to the :ref:`template-configuration-label` section.

Copy and paste the content from above into a file and save it as ``template.json``
.

Inventory
~~~~~~~~~

Next, ``scdetect-cc`` requires :external:term:`inventory` data to be provided.
Simply download the required :external:term:`inventory` data with the command below:

.. code-block:: bash

   echo "http://eida.ethz.ch/fdsnws/station/1/query"\
   "?net=CH&sta=SENIN&channel=HH?&level=response" | \
   wget -i - -O stations.xml

Now, convert the ``station.xml`` file
into :external:term:`SCML` format:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec fdsnxml2inv -f stations.xml > stations.scml

Template EventParameters
~~~~~~~~~~~~~~~~~~~~~~~~

Apart from the :external:term:`inventory` data ``scdetect-cc`` requires *EventParameter*
configuration (:external:term:`SCML` formatted) describing the template events.

Depending on the use case this configuration file must include:


* Picks ``-P``
* StationMagnitudes ``-M``
* Amplitudes ``-A``

For downloading the data from the database we
use :external:ref:`scxmldump`:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scxmldump \
     -d localhost \
     -O smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990 \
     -f \
     -M \
     -P \
     -A \
     -o template.scml

.. note::

   ``-d localhost`` refers to your database. It can be something like
   ``postgresql://user:password@host/seiscompDbName``


Here, template EventParameter configuration is downloaded for a single template
configuration (i.e. identified by the ``detectorId`` ``detector-01``\ ). The
corresponding template ``originId``
is ``smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990``.

.. note::

   At the time being you cannot download station magnitudes from
   SeisComP's ``fdsnws-event`` service implementation. For the purpose of this
   tutorial you can get the template EventParameter configuration from
   `here <https://github.com/swiss-seismological-service/scdetect/tree/master/doc/data/Offline-Playback/ex-01/template.scml>`__.
   Of course, you can also modify this tutorial to match your needs.


Waveform data
~~~~~~~~~~~~~


* 
  **Template waveform data**

  To make things run faster we will create template waveforms. Once you have the
  station :external:term:`inventory` (``stations.scml``), the template EventParameter
  configuration (``template.scml``), and the template configuration
  (``template.json``) ready, invoke the following command:

  .. code-block:: bash

     $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
       --offline \
       --amplitudes-force=0 \
       --debug \
       --templates-json template.json \
       --inventory-db stations.scml \
       --event-db template.scml \
       --record-url "fdsnws://eida.ethz.ch/fdsnws/dataselect/1/query" \
       --templates-prepare

  The command above will download the required data for template waveform
  creation from ``http://eida.ethz.ch/fdsnws/dataselect/1/query``\ , and store it in
  a local cache.

* 
  **Raw continuous waveform data**

  Next, download the waveform data to be processed. Here, we will download the
  data from a FDSN service by means of ``wget``\ , and sort it on-the-fly:

  .. code-block:: bash

     echo "http://eida.ethz.ch/fdsnws/dataselect/1/query"\
     "?net=CH&sta=SENIN&channel=HH?&start=2019-11-05T04:00&end=2019-11-05T05:00" | \
     wget -i - -O - | \
     $SEISCOMP_ROOT/bin/seiscomp exec scmssort -u -E -v - > data.mseed

  .. note::

     When processing data from multiple streams in *playback mode* it is
     important to sort data by end time.


.. _ex-01-configure-bindings-label:

Configure Bindings
~~~~~~~~~~~~~~~~~~

Bindings configuration allows you to provide dedicated configuration on station
granularity. In the context of ``scdetect-cc`` bindings configuration is required
in order to successfully compute
both :ref:`amplitudes <theory-amplitude-calculation-label>` and
estimate :ref:`magnitudes <theory-magnitude-estimation-label>`.

Please refer to the
official :external:ref:`SeisComP documentation <global_bindings_config>`
for further information on bindings configuration.

SeisComP Experts
""""""""""""""""

If you have already configured the bindings then you simply need to export those
with e.g.

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec bindings2cfg \
     --key-dir $SEISCOMP_ROOT/etc/key \
     -o bindings.scml

SeisComP Newbies
""""""""""""""""

If you're new to SeisComP you can follow this guide in order to create your
:external:term:`SCML`
formatted :external:ref:`bindings configuration <global_bindings_config>`
file:


#. 
   First of all, import the previously
   downloaded ``station.xml`` :external:term:`inventory` file to ``SeisComP``:

   .. code-block:: bash

      $SEISCOMP_ROOT/bin/seiscomp exec import_inv fdsnxml stations.xml

#. 
   Next, synchronize
   the :external:term:`inventory` with the following command:

   .. code-block:: bash

       $SEISCOMP_ROOT/bin/seiscomp exec scinv sync

   .. note::

      This operation may change possibly preexisting :external:term:`inventory`
      data on your database.


#. 
   Now, let's create and assign the bindings configuration. For simplicity,
   open :external:ref:`scconfig`

   .. code-block:: bash

       $SEISCOMP_ROOT/bin/seiscomp exec scconfig &

   Then navigate to the :external:ref:`bindings panel <scconfig-bindings>` [1]
   . In the *module tree* select *scdetect-cc* [2] in the upper area,
   right-click and select *Add scdetect-cc profile*.


   .. image:: media/Offline-Playback/ex-01/bindings_panel.png
    :width: 600
    :align: center
    :alt: Bindings panel

   A popup will appear which asks you to name the *binding profile* to be
   created. Name the binding profile ``default`` or whatever you like and set the
   configuration parameters. An example for computing MRelative magnitudes is


   .. image:: media/Offline-Playback/ex-01/scdetect_cc_bindings_config.png
    :width: 600
    :align: center
    :alt: Bindings configuration

   Then, drag and drop the profile to the ``CH`` network. With that, the binding
   profile is going to be assigned to all stations which are part of the ``CH``
   network.

#. 
   Next, save the configuration bindings to the database. Navigate
   to :external:ref:`scconfig` \ 's :external:ref:`system panel <scconfig-system>`
   and press the *Update configuration* button in the toolbar on the top.

#. 
   Finally, export the bindings configuration from the database to
   the ``bindings.scml`` file:

   .. code-block:: bash

       $SEISCOMP_ROOT/bin/seiscomp exec bindings2cfg \
          --key-dir $SEISCOMP_ROOT/etc/key \
          -o bindings.scml

Lazy Users
""""""""""

Get the bindings configuration
from `here <https://github.com/swiss-seismological-service/scdetect/tree/master/doc/data/Offline-Playback/ex-01/bindings.scml>`__.

Execute
~~~~~~~

Now, let's detect earthquakes:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
     --offline \
     --playback \
     --debug \
     --templates-json template.json \
     --inventory-db stations.scml \
     --event-db template.scml \
     --config-db bindings.scml \
     --record-url data.mseed \
     --ep=results.scml

The detection results will be output in
the :external:term:`SCML` formatted ``results.scml`` file.

.. _ex-02-label:

Example: Single detector - Multi-station - Multi-channel detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While in
the :ref:`previous example <ex-01-label>`
the template configuration used only streams from a single station, this
tutorial will guide you to detect earthquakes using two stations (each with
three channels).

Setup
~~~~~

Create an empty directory with

.. code-block:: bash

   mkdir -p $HOME/tmp/scdetect-cc/ex-02 && cd  $HOME/tmp/scdetect-cc/ex-02

Template Configuration
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: json

   [
     {
       "detectorId": "detector-01",
       "createArrivals": true,
       "createTemplateArrivals": true,
       "createAmplitudes": true,
       "createMagnitudes": true,
       "gapInterpolation": true,
       "gapThreshold": 0.1,
       "gapTolerance": 1.5,
       "triggerDuration": 0.5,
       "triggerOnThreshold": 0.4,
       "triggerOffThreshold": 0.0,
       "arrivalOffsetThreshold": 1.0,
       "minimumArrivals": 3,
       "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990",
       "filter": "BW_BP(2,1.5,15)",
       "templateFilter": "BW_BP(2,1.5,15)",
       "streams": [
         {
           "templateId": "template-01",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SENIN..HHZ",
           "templateWaveformId": "CH.SENIN..HHZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-02",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHN",
           "templateWaveformId": "CH.SENIN..HHN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-03",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHE",
           "templateWaveformId": "CH.SENIN..HHE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-04",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SAYF2..HGZ",
           "templateWaveformId": "CH.SAYF2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-05",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGE",
           "templateWaveformId": "CH.SAYF2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-06",
           "initTime": 10,
           "templateWaveformStart": -0.5,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGN",
           "templateWaveformId": "CH.SAYF2..HGN",
           "templatePhase": "Sg"
         }
       ]
     }
   ]

Above a template configuration based on the template event identified by
the ``originId`` ``smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990``
covering 6 streams overall. While the streams ``HHZ``\ , ``HHN``\ , and ``HHE`` refer to
the sensor location ``CH.SENIN..``\ , the sensor location ``CH.SAYF2..`` is covered by
the streams ``HGZ``\ , ``HGN``\ , and ``HGE``. Again, the template waveforms of the
vertical instrument components refer to the ``templatePhase`` ``Pg``\ , while the
template waveforms of the horizontal instrument components refer to
the ``templatePhase`` ``Sg``.

.. note::

   In contrast to the `previous example <ex-01-label>`, here, we introduce the
   ``minimumArrivals`` configuration option, which defines the minimum number
   of phases required to declare a new detection. The default value is the
   total number of phase templates, i.e. 6 in this case.


For a more detailed description of the available configuration options please
refer to the :ref:`template-configuration-label`
section.

Copy and paste the content from above into a file and save it as ``template.json``
.

Inventory
~~~~~~~~~

Simply download the
required :external:term:`inventory` data with the command below:

.. code-block:: bash

   echo "http://eida.ethz.ch/fdsnws/station/1/query"\
   "?net=CH&sta=SENIN,SAYF2&channel=HH?,HG?&level=response" | \
   wget -i - -O stations.xml

Then, convert the ``station.xml`` file into
the :external:term:`SCML` format:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec fdsnxml2inv -f stations.xml > stations.scml

Template EventParameters
~~~~~~~~~~~~~~~~~~~~~~~~

Download the template EventParameter data for the
template ``originId`` ``smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990``
from the database with :external:ref:`scxmldump`:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scxmldump \
     -d localhost \
     -O smi:ch.ethz.sed/sc3a/origin/NLL.20191105125505.255283.1897990 \
     -f \
     -M \
     -P \
     -A \
     -o template.scml

For the purpose of this tutorial you can download the template EventParameter
data from
`here
<https://github.com/swiss-seismological-service/scdetect/tree/master/doc/data/Offline-Playback/ex-02/template.scml>`__,
since you don't have access to the database.

Waveform data
~~~~~~~~~~~~~


* 
  **Template waveform data**

  To make things run faster we will create template waveforms. Once you have
  the :external:term:`inventory` (\ ``stations.scml``\ )
  , the template EventParameter configuration (\ ``template.scml``\ ), and the
  template configuration (\ ``template.json``\ ) ready, invoke the following command:

  .. code-block:: bash

       $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
           --offline \
           --templates-prepare \
           --amplitudes-force=0  \
           --debug \
           --templates-json template.json \
           --inventory-db stations.scml \
           --event-db template.scml \
           --record-url "fdsnws://eida.ethz.ch/fdsnws/dataselect/1/query"

  This will download the required data
  from ``http://eida.ethz.ch/fdsnws/dataselect/1/query`` and store it in a local
  cache.

* 
  **Raw continuous waveform data**

  The waveform data to be precessed may be downloaded and sorted on-the-fly with
  e.g.:

  .. code-block:: bash

     echo "http://eida.ethz.ch/fdsnws/dataselect/1/query"\
     "?net=CH&sta=SENIN,SAYF2&cha=HH?,HG?"\
     "&start=2019-11-05T04:00:00&end=2019-11-05T05:00:00" | \
     wget -i - -O - | \
     $SEISCOMP_ROOT/bin/seiscomp exec scmssort -u -E -v - > data.mseed

Configure Bindings
~~~~~~~~~~~~~~~~~~

In order to successfully compute
both :ref:`amplitudes <theory-amplitude-calculation-label>` and
estimate :ref:`magnitudes <theory-magnitude-estimation-label>`
``scdetect-cc`` requires bindings configuration.

Please refer to the :ref:`previous tutorial <ex-01-configure-bindings-label>` for a detailed
description on how to create the bindings configuration file. Alternatively,
download the bindings configuration file
from `here <https://github.com/swiss-seismological-service/scdetect/tree/master/doc/data/Offline-Playback/ex-02/bindings.scml>`__.

Execute
~~~~~~~

Now, let's detect earthquakes:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
       --offline \
       --playback \
       --debug \
       --templates-json template.json \
       --inventory-db stations.scml \
       --event-db template.scml \
       --config-db bindings.scml \
       --record-url data.mseed \
       --ep=results.scml

Post-processing
~~~~~~~~~~~~~~~

``scdetect-cc`` generates origins, picks, arrivals, amplitudes, and both station
magnitudes and network magnitudes. When using multiple detectors, the
resulting ``results.scml`` file will, in theory at least, contain earthquakes
detected from different templates (i.e. detectors). An easy way to merge these
detections into events is to use :external:ref:`scevent`.
Before, make sure you've set the desired ``eventAssociation`` related module
configuration parameters, e.g.


.. image:: media/Offline-Playback/ex-02/scevent_module_config.png
  :width: 600
  :align: center
  :alt: scevent module configuration


Then, run :external:ref:`scevent`:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scevent \
     --debug \
     --disable-db \
     --ep results.scml > event.catalog.scml

Screen results
~~~~~~~~~~~~~~

The last step is to visualize the results. First we need to
launch :external:ref:`scolv`:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scolv \
     --offline \
     --debug \
     --db-disable \
     --inventory-db stations.scml \
     --record-url "fdsnws://eida.ethz.ch/fdsnws/dataselect/1/query"  &

Here, we run :external:ref:`scolv` without
using a database. Besides, waveform data is fetched from FDSN web services.

Once :external:ref:`scolv` is up and running
import the ``event.catalog.scml`` file which was previously generated
with :external:ref:`scevent`.

.. _ex-03-label:

Example: Multi-detector - Multi-station - Multi-channel detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tutorial will guide you to detect earthquakes using three stations (i.e.
each with three channels), and multiple earthquake templates (i.e. ``detector-01``
, ``detector-02``\ , etc).

Setup
~~~~~

Create an empty directory with

.. code-block:: bash

   mkdir -p $HOME/tmp/scdetect-cc/ex-03 && cd  $HOME/tmp/scdetect-cc/ex-03

Template Configuration
~~~~~~~~~~~~~~~~~~~~~~

Copy and paste the following template configuration into a file and save it
as ``template.json``.

.. code-block:: json

   [
     {
       "detectorId": "detector-01",
       "createArrivals": true,
       "createTemplateArrivals": true,
       "createAmplitudes": true,
       "createMagnitudes": true,
       "gapInterpolation": true,
       "gapThreshold": 0.1,
       "gapTolerance": 1.5,
       "minimumArrivals": 5,
       "triggerDuration": 0.5,
       "triggerOnThreshold": 0.5,
       "triggerOffThreshold": 0.0,
       "arrivalOffsetThreshold": 0.5,
       "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20191105141052.482388.186933",
       "filter": "BW_BP(2,1.5,15)",
       "templateFilter": "BW_BP(2,1.5,15)",
       "streams": [
         {
           "templateId": "template-01",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SAYF2..HGZ",
           "templateWaveformId": "CH.SAYF2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-02",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGE",
           "templateWaveformId": "CH.SAYF2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-03",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGN",
           "templateWaveformId": "CH.SAYF2..HGN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-04",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.STSW2..HGZ",
           "templateWaveformId": "CH.STSW2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-05",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.STSW2..HGE",
           "templateWaveformId": "CH.STSW2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-06",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.STSW2..HGN",
           "templateWaveformId": "CH.STSW2..HGN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-07",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SENIN..HHZ",
           "templateWaveformId": "CH.SENIN..HHZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-08",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHE",
           "templateWaveformId": "CH.SENIN..HHE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-09",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHN",
           "templateWaveformId": "CH.SENIN..HHN",
           "templatePhase": "Sg"
         }
       ]
     },
     {
       "detectorId": "detector-02",
       "createArrivals": true,
       "createTemplateArrivals": true,
       "createAmplitudes": true,
       "createMagnitudes": true,
       "gapInterpolation": true,
       "gapThreshold": 0.1,
       "gapTolerance": 1.5,
       "minimumArrivals": 8,
       "triggerDuration": 0.5,
       "triggerOnThreshold": 0.5,
       "triggerOffThreshold": 0.0,
       "arrivalOffsetThreshold": 0.3,
       "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20191106214318.866617.258170",
       "filter": "BW_BP(2,1.5,15)",
       "templateFilter": "BW_BP(2,1.5,15)",
       "streams": [
         {
           "templateId": "template-01",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SAYF2..HGZ",
           "templateWaveformId": "CH.SAYF2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-02",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGE",
           "templateWaveformId": "CH.SAYF2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-03",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGN",
           "templateWaveformId": "CH.SAYF2..HGN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-04",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.STSW2..HGZ",
           "templateWaveformId": "CH.STSW2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-05",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.STSW2..HGE",
           "templateWaveformId": "CH.STSW2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-06",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.STSW2..HGN",
           "templateWaveformId": "CH.STSW2..HGN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-07",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SENIN..HHZ",
           "templateWaveformId": "CH.SENIN..HHZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-08",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHE",
           "templateWaveformId": "CH.SENIN..HHE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-09",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHN",
           "templateWaveformId": "CH.SENIN..HHN",
           "templatePhase": "Sg"
         }
       ]
     },
     {
       "detectorId": "detector-03",
       "createArrivals": true,
       "createTemplateArrivals": true,
       "createAmplitudes": true,
       "createMagnitudes": true,
       "gapInterpolation": true,
       "gapThreshold": 0.1,
       "gapTolerance": 1.5,
       "minimumArrivals": 8,
       "triggerDuration": 0.5,
       "triggerOnThreshold": 0.5,
       "triggerOffThreshold": 0.0,
       "arrivalOffsetThreshold": 0.3,
       "originId": "smi:ch.ethz.sed/sc3a/origin/NLL.20191106214119.274905.257656",
       "filter": "BW_BP(2,1.5,15)",
       "templateFilter": "BW_BP(2,1.5,15)",
       "streams": [
         {
           "templateId": "template-01",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SAYF2..HGZ",
           "templateWaveformId": "CH.SAYF2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-02",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGE",
           "templateWaveformId": "CH.SAYF2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-03",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SAYF2..HGN",
           "templateWaveformId": "CH.SAYF2..HGN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-04",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.STSW2..HGZ",
           "templateWaveformId": "CH.STSW2..HGZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-05",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.STSW2..HGE",
           "templateWaveformId": "CH.STSW2..HGE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-06",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.STSW2..HGN",
           "templateWaveformId": "CH.STSW2..HGN",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-07",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 1,
           "waveformId": "CH.SENIN..HHZ",
           "templateWaveformId": "CH.SENIN..HHZ",
           "templatePhase": "Pg"
         },
         {
           "templateId": "template-08",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHE",
           "templateWaveformId": "CH.SENIN..HHE",
           "templatePhase": "Sg"
         },
         {
           "templateId": "template-09",
           "initTime": 10,
           "templateWaveformStart": -0.25,
           "templateWaveformEnd": 2,
           "waveformId": "CH.SENIN..HHN",
           "templateWaveformId": "CH.SENIN..HHN",
           "templatePhase": "Sg"
         }
       ]
     }
   ]

Above a template configuration for the multiple template events. The template
event identified by ``detectorId`` ``detector-01`` is based on
the ``originId`` ``smi:ch.ethz.sed/sc3a/origin/NLL.20191105141052.482388.186933``\ ,
the template event identified by ``detectorId`` ``detector-02`` is based on
the ``originId`` ``smi:ch.ethz.sed/sc3a/origin/NLL.20191106214318.866617.258170``\ ,
and the template identified by ``detectorId`` ``detector-03`` is based on
the ``originId`` ``smi:ch.ethz.sed/sc3a/origin/NLL.20191106214119.274905.257656``.
All template event configurations use waveform data from the sensor
locations ``CH.SAYF2..``\ , ``CH.SENIN..``\ , and ``CH.STSW2..``.

Inventory
~~~~~~~~~

Download
the :external:term:`inventory` data with the following command:

.. code-block:: bash

   echo "http://eida.ethz.ch/fdsnws/station/1/query"\
   "?net=CH&sta=SENIN,SAYF2,STSW2&channel=H??&level=response" | \
   wget -i - -O stations.xml

Now, convert the ``station.xml`` file
into :external:term:`SCML` format:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec fdsnxml2inv -f stations.xml > stations.scml

Template EventParameters
~~~~~~~~~~~~~~~~~~~~~~~~

Download the template EventParameter data for the template ``originId``
s ``smi:ch.ethz.sed/sc3a/origin/NLL.20191105141052.482388.186933``
, ``smi:ch.ethz.sed/sc3a/origin/NLL.20191106214318.866617.258170``\ ,
and ``smi:ch.ethz.sed/sc3a/origin/NLL.20191106214119.274905.257656`` from the
database with :external:ref:`scxmldump`:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scxmldump \
     -d localhost \
     -O smi:ch.ethz.sed/sc3a/origin/NLL.20191105141052.482388.186933,smi:ch.ethz.sed/sc3a/origin/NLL.20191106214318.866617.258170,smi:ch.ethz.sed/sc3a/origin/NLL.20191106214119.274905.257656 \
     -f \
     -M \
     -P \
     -A \
     -o template.scml

For the purpose of this tutorial you can download the template EventParameter
data from
`here
<https://github.com/swiss-seismological-service/scdetect/tree/master/doc/data/Offline-Playback/ex-03/template.scml>`__,
since you don't have access to the database.

Waveform data
~~~~~~~~~~~~~


* 
  **Template waveform data**

  Download and prepare the template waveform data with the following command:

  .. code-block:: bash

       $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
         --offline \
         --templates-prepare \
         --amplitudes-force=0 \
         --debug \
         --templates-json template.json \
         --inventory-db stations.scml \
         --event-db template.scml \
         --record-url "fdsnws://eida.ethz.ch/fdsnws/dataselect/1/query"

  Note that ``scdetect-cc`` will store the downloaded waveform data within a local
  cache.

* 
  **Raw continuous waveform data**

  The waveform data to be precessed may be downloaded and sorted on-the-fly with
  e.g.:

  .. code-block:: bash

     echo "http://eida.ethz.ch/fdsnws/dataselect/1/query"\
     "?net=CH&sta=SENIN,SAYF2,STSW2&cha=HH?,HG?"\
     "&start=2019-11-05T04:00:00&end=2019-11-05T05:00:00" | \
     wget -i - -O - | \
     $SEISCOMP_ROOT/bin/seiscomp exec scmssort -u -E -v - > data.mseed

Configure Bindings
~~~~~~~~~~~~~~~~~~

In order to successfully compute
both :ref:`amplitudes <theory-amplitude-calculation-label>` and
estimate :ref:`magnitudes <theory-magnitude-estimation-label>`
``scdetect-cc`` requires bindings configuration.

Please refer to the :ref:`first tutorial <ex-01-configure-bindings-label>` for a detailed
description on how to create the bindings configuration file. Alternatively,
download the bindings configuration file
from `here <https://github.com/swiss-seismological-service/scdetect/tree/master/doc/data/Offline-Playback/ex-03/bindings.scml>`__.

Execute
~~~~~~~

Now, let's detect earthquakes:

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
     --offline \
     --playback \
     --debug \
     --templates-json template.json \
     --inventory-db stations.scml \
     --event-db template.scml \
    --config-db bindings.scml \
    --record-url data.mseed \
     --ep=results.scml

``scdetect-cc`` will dump the results to the ``results.scml`` file.

Post-processing
~~~~~~~~~~~~~~~

Run :external:ref:`scevent` with the
following command in order to associate the origins from the ``results.scml`` file
to events.

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scevent \
     --debug \
     --disable-db \
     --ep results.scml > event.catalog.scml

This will create a catalog which is stored in the ``event.catalog.scml`` file.

Screen results
~~~~~~~~~~~~~~

To screen visualize the results launch :external:ref:`scolv` with

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scolv \
     --offline \
     --db-disable \
     --debug \
     --inventory-db stations.scml \
     --record-url "fdsnws://eida.ethz.ch/fdsnws/dataselect/1/query" &

Once :external:ref:`scolv` is up and running
import the ``event.catalog.scml`` file which was previously generated
with :external:ref:`scevent`.
