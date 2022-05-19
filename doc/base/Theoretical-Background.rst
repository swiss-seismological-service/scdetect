.. _theoretical-background-label:

Theoretical Background
======================

.. _theory-earthquake-detection-label:

Earthquake detection
--------------------

Here describe the detection process

.. _theory-phase-association-label:

Phase association
-----------------

Here describe the Linker

.. _theory-amplitude-calculation-label:

Amplitude calculation
---------------------

Computing amplitudes is a prerequisite in order to perform a magnitude
estimation later on. Since multiple magnitude estimation methods are provided,
each magnitude estimation method requires to compute a corresponding amplitude
type. In accordance with the magnitudes methods described in
the :ref:`magnitude estimation <theory-magnitude-estimation-label>` section ``scdetect-cc``
implements the following amplitude types to be computed:


* 
  ``MRelative``\ : Amplitude computed as the ratio between the template waveform and
  the detection. The approach is outlined by
  e.g. https://doi.org/10.1038/ngeo697
  and https://doi.org/10.1126/sciadv.1601946 and uses the same instrument
  components as specified by the detector configuration.

* 
  ``MLx``\ : Amplitudes required for the *amplitude-magnitude regression* approach.
  The implementation follows the approach outlined
  in https://doi.org/10.1029/2019JB017468 (section 3.3.3 *Magnitude Estimation*\ )
  . Amplitudes used for the amplitude-magnitude regression are so called *sensor
  location* `RMS (root-mean-square) <https://en.wikipedia.org/wiki/Root_mean_square>`_
  *amplitudes* (i.e. the maximum sample-wise RMS regarding the horizontal
  components for a certain sensor location w.r.t. velocity seismograms).

Amplitudes are calculated once an origin has been declared.

In general, the computation of amplitudes is sensor location dependent. In order
to provide dedicated configuration for different sensor locations ``scdetect-cc``
makes use of
SeisComP's :external:ref:`bindings configuration <global_bindings_config>`
concept. Note that amplitudes are calculated only:


* for those sensor locations with bindings configuration available,
* if the internal waveform buffer still contains the required time window.

The waveform buffer size may be configured using
the ``processing.waveformBufferSize`` module configuration parameter.

.. _theory-magnitude-estimation-label:

Magnitude estimation
--------------------

``scdetect-cc`` estimates magnitudes as so called SeisComP *station magnitudes* (
for further details, please refer to the :external:ref:`scmag documentation <scmag>`)
. Magnitudes may be estimated for only those sensor locations, the corresponding
magnitude types were computed, previously. In accordance with the amplitude
types described in the :ref:`amplitude calculation section <theory-amplitude-calculation-label>`,
the following magnitude types are available:


* 
  ``MRelative``\ : Template-detection ratio based magnitude estimation. Besides, of
  the corresponding amplitudes to be computed, this particular type requires
  station magnitudes to be available
  through :ref:`EventParameters <inventory-events-and-configuration-label>`.
  (\ **References**\ : https://doi.org/10.1038/ngeo697
  , https://doi.org/10.1126/sciadv.1601946)

* 
  ``MLx``\ : Amplitude-magnitude regression based magnitude type. Besides, of the
  corresponding amplitudes to be computed, this particular type requires both
  amplitudes and station magnitudes to be available by means
  of :ref:`EventParameters <inventory-events-and-configuration-label>`. Moreover, the
  approach is based on so-called *template families* which in fact are groups of
  *related* templates. The
  corresponding :ref:`template family configuration <template-family-configuration-label>`
  must be provided by ``scdetect-cc``\ '
  s ``--templates-family-json path/to/templates-family.json`` CLI flag.
  (\ **References**\ : https://doi.org/10.1029/2019JB017468 (section 3.3.3
  *Magnitude Estimation*\ ))

All magnitude estimation methods listed above are based on the following types
of *template station magnitudes*\ :


* 
  ``MLh``: please refer to the :ref:`SeisComP documentation <global_mlh>`

* 
  ``MLhc``\ : based on ``MLh``\ , but uses a slightly adjusted relationship (i.e.
  corrected for near-field observations) and allows for station specific
  corrections.

..

   **NOTE**\ : Magnitudes of type ``MLhc`` are preferred over magnitudes of type ``MLh``.


Recall, that template station magnitudes must be available through event
parameters (for further details, please refer to the
related :ref:`section <inventory-events-and-configuration-label>` on providing these data
products).
