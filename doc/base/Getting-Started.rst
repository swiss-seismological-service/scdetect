.. _getting-started-label:

Getting Started
===============

New to SCDetect? Don't worry, you've found the perfect place to get started.

We assume that SeisComP and SCDetect are successfully installed on your
machine. For installation instructions, please refer to the `SCDetect project page <https://github.com/swiss-seismological-service/scdetect/tree/master#compiling-and-installation>`_.

In order to get quickly started, we've prepared some exemplary data including
the required configuration. Simply download it
`here <https://github.com/swiss-seismological-service/scdetect/doc/data/getting.started.tar.xz>`_.

Once downloaded, extract the ``getting.started.tar.xz`` file and change into the
directory

.. code-block:: bash

   tar -xvJf getting.started.tar.xz && cd getting.started

Then, run ``scdetect-cc`` in offline mode using the following command

.. code-block:: bash

   $SEISCOMP_ROOT/bin/seiscomp exec scdetect-cc \
     --offline \
     --playback \
     --debug \
     --templates-json templates.json \
     --inventory-db stations.scml \
     --event-db template.scml \
     --record-url data.mseed \
     --config-db bindings.scml \
     --ep=results.scml

Congratulations. You have successfully processed the data from the ``data.mseed``
file. The results should be available in the generated ``results.scml`` file.

For a more detailed description regarding the downloaded files and configuration
please refer to the ``readme.txt`` (located in the ``getting.started``
folder).

If you're interested in how to customize the configuration and how to prepare
the data please refer to our more in-depth :ref:`scdetect-for-dummies-label`
guide.
