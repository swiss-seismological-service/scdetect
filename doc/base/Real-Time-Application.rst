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
