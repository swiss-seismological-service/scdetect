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
declares a new :external:term:`origin`.

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

:raw-html-m2r:`<em>Mesimeri, M., Armbruster, D., Kästli, P., Scarabello, L., Diehl, T., 
Clinton, J., Wiemer, S.</em>` (2024)\ :raw-html-m2r:`<br>`
SCDetect: A SeisComP Module for Real‐Time Waveform Cross‐Correlation‐Based Earthquake 
Detection\ :raw-html-m2r:`<br>`
:raw-html-m2r:`<em>Seismological Research Letters, 95 (3): 1961–1975</em>`\ ,
DOI: https://doi.org/10.1785/0220230164

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
