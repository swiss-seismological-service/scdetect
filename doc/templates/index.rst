.. role:: raw-html-m2r(raw)
   :format: html

About
=====

SCDetect is a `SeisComP <https://github.com/SeisComP>`_ :cite:p:`seiscomp`
package. With the extension module ``scdetect-cc`` it implements both real-time
and classical offline earthquake detection based on waveform cross-correlation,
also called matched filtering or template matching. Again, the underlying
cross-correlation algorithm is based on computing
the `Pearson Correlation Coefficient <https://en.wikipedia.org/wiki/Pearson_correlation_coefficient>`_
.

The module allows both single-stream and multi-stream earthquake detection.

In case the detection parameters exceed the configured thresholds, ``scdetect-cc``
declares a new origin.

Besides, magnitudes may be estimated based on multiple magnitude estimation
methods (i.e. regression, amplitude ratios).

Installation
============

Instructions for compiling and the installation of the software can be found on
the
`SCDetect project page
<https://github.com/swiss-seismological-service/scdetect#compiling-and-installation>`_.

Cite
====

If you intend to use SCDetect please cite as:

:raw-html-m2r:`<em>Armbruster, D., Mesimeri, M., Kästli, P., Diehl, T., Massin, F., and Wiemer,
S.</em>` (2022)\ :raw-html-m2r:`<br>`
SCDetect: Near real-time computationally efficient waveform cross-correlation
based earthquake detection during intense earthquake sequences\ :raw-html-m2r:`<br>`
:raw-html-m2r:`<em>EGU General Assembly 2022, Vienna, Austria, 23–27 May 2022</em>`\ ,
EGU22-12443\ :raw-html-m2r:`<br>`
DOI: https://doi.org/10.5194/egusphere-egu22-12443

.. note::

   A manuscript is currently in preparation.


Help & Feedback
===============

Contact us:

*Daniel Armbruster*\ : daniel.armbruster [at]
sed.ethz.ch `GitHub <https://github.com/damb>`_

*Maria Mesimeri*\ : maria.mesimeri [at]
sed.ethz.ch `Github <https://github.com/mmesim>`_ `Twitter <https://twitter.com/QuakeMary>`_ `Gscholar <https://scholar.google.gr/citations?user=Za5cbYUAAAAJ&hl=en>`_


.. toctree::
   :hidden:
   :glob:
   :maxdepth: 2
   :caption: Getting Started

   /base/Getting-Started
   /base/SCDetect-for-Dummies

.. toctree::
   :hidden:
   :glob:
   :maxdepth: 2
   :caption: Configuration

   scdetect-cc configuration parameters </apps/scdetect-cc>
   /base/Template-Configuration
   /base/Bindings-Configuration-Details
   /base/Template-Family-Configuration

.. toctree::
   :hidden:
   :glob:
   :maxdepth: 2
   :caption: Background

   /base/Theoretical-Background
   /base/Architecture
   /base/Data-and-Resources

.. toctree::
   :hidden:
   :glob:
   :maxdepth: 2
   :caption: References

   /base/References
