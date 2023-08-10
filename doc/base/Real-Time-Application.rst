.. _real-time-application-label:

Real-Time Application
---------------------


We assume that you're familiar with `SeisComP <https://www.seiscomp.de/>`_ and its
underlying concepts.

This section will describe step-by-step the prerequisites to be fulfilled in
order to process waveform data with ``scdetect-cc`` in real-time fashion.

Inventory
^^^^^^^^^

We assume that
the :external:term:`inventory` data of interest is available.

Bindings Configuration
^^^^^^^^^^^^^^^^^^^^^^

:external:ref:`Bindings configuration <global_bindings_config>`
for the stations of interest is required:


* 
  Global bindings are required for each sensor in order to display the waveform
  data with :external:ref:`scrttv`.

* 
  Also, create :external:ref:`seedlink`
  bindings configuration with the correct source plugin enabled.

* 
  Finally, create bindings configuration for ``scdetect-cc`` in order to be able
  to successfully compute
  both :ref:`amplitudes <theory-amplitude-calculation-label>` and
  estimate :ref:`magnitudes <theory-magnitude-estimation-label>`.

Template EventParameter
^^^^^^^^^^^^^^^^^^^^^^^

Template EventParameter data needs to be available for ``scdetect-cc``.

SeisComP configuration
^^^^^^^^^^^^^^^^^^^^^^

Make sure the module configuration for the used modules is configured correctly.
Particularly, check the ``scdetect-cc`` module configuration parameters.

Real-time monitoring and detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a few GUIs to monitor :external:ref:`scmaster` and events

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scrttv & \
   $SEISCOMP_ROOT/bin/seiscomp exec scmm & \
   $SEISCOMP_ROOT/bin/seiscomp exec scesv & \
   $SEISCOMP_ROOT/bin/seiscomp exec scolv &

and invoke

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
     --start-stop-msg=1 \
     --debug

to run ``scdetect-cc``. Alternatively, start ``scdetect-cc`` in daemon mode.

Update Templates in Real Time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here we briefly describe how to manage ``templates`` traffic in real time. 

* 1.  Download earthquakes around the Mainshock / Earthquake of interest

   We assume that an earthquake occurred in the are and we are interested in 
   finding past [or new] earthquakes to use as ``detectors``. Then using
   `FDSN services <https://www.fdsn.org/webservices/>`_ and `obspy <https://docs.obspy.org/>`_ 
   we get the event catalog that contains arrival times: 
               
    .. code-block:: python 
                      
       cat = client.get_events(latitude=latitude, longitude=longitude, 
                               minradius=0.001, maxradius=0.2,
                               eventtype='earthquake', limit='30',
                               contributor="SED", minmagnitude=1.0,
                               includearrivals=True)
          

* 2. Create ``JSON`` file with ``detectors`` and ``templates``
   
   A python script is included in `scdetect-cc <https://github.com/swiss-seismological-service/scdetect/blob/master/src/apps/scripts/catalog2templates.py>`_. 
   This script allows the user to convert the event file into ``JSON``. 

   Redirect the output to the path define in :external:ref:`scconfig` . 

* 3. Cache ``templates`` waveforms      
   
   Folow the instructions from `here <https://scdetect.readthedocs.io/en/stable/base/SCDetect-for-Dummies.html#waveform-data>`_ 
   to cache the template waveforms. 

* 4. Run ``scdetect-cc``
        
   The last step is to restart ``scdetect-cc`` 

   .. code-block:: bash

      $SEISCOMP_ROOT/bin/seiscomp restart scdetect-cc 


All the above steps can be combined into a single script (e.g. ``bash``), and setup a cronjob
that will update the ``templates.json`` file every few minutes or hours.  

``date`` commnad in ``bash`` can be useful to set up a time period.

:external:ref:`scevent` takes care of event association.

