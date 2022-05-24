.. _bindings-configuration-label:

Bindings configuration details
==============================

This section contains a detailed description how to set up the configuration
required for amplitude calculation.

For those users already familiar with
SeisComP's :external:ref:`bindings configuration <global_bindings_config>`
an important note:

..

   **NOTE**\ : At the time being, SeisComP's
   :external:ref:`bindings configuration <global_bindings_config>`
   allows configuration to be provided for stations, only. In order to allow
   users to supply configuration on sensor location granularity, ``scdetect-cc``
   makes use of so called *sensor location profiles*.


In general, bindings are configured the easiest with SeisComP's configuration
and system management frontend :external:ref:`scconfig <scconfig-bindings>`
. Alternatively, *key files* may be created manually.

**Creating key files** (\ ``scconfig``\ ):


#. 
   Create an *sensor location profile* for a sensor location. As a bare minimum
   specify at least the ``locationCode`` (which may be empty) and ``channelCode``
   attributes.

   **Tip**\ : Although not strictly required, it is recommended to use sensible
   profile names. E.g. for a sensor location profile with ``locationCode`` ``00``
   and ``channelCode`` ``HH`` naming the profile with e.g. ``00_HH`` is recommended.

   In both the ``locationCode`` and the ``channelCode`` the wildcard
   characters ``?`` (which matches any single character) and ``*`` (which matches
   zero to many characters) are allowed.

   **Tip**\ : Make use of wildcard characters in order to create a *default*
   sensor location profile.

#. 
   Add the profile's name to the list of known ``sensorLocationProfiles``. Only
   those profiles are taken into account with a corresponding list entry.

#. 
   (Optional): in case of creating a *binding profile* assign the bindings
   configuration to the corresponding stations.

#. 
   Save the configuration. With that, the bindings configuration is written to
   so called *key files*.

**Dump bindings configurations**\ :

``scdetect-cc`` (and SeisComP :external:term:`trunk modules <trunk>`
in general) do not work with key files directly. Therefore, key files need to be
dumped either to a database or into
a :external:term:`SCML`
formatted configuration file (depending on whether a file-based approach is
desired or not). This is done the easiest with SeisComP's
:external:ref:`bindings2cfg` utility. E.g.

.. code-block:: bash

   $ bindings2cfg --key-dir path/to/key -o config.scml

will dump the configuration into
a :external:term:`SCML` formatted ``config.scml`` file, while

.. code-block:: bash

   $ bindings2cfg --key-dir path/to/key \
     --plugins dbpostgresql -d postgresql://user:password@localhost/seiscomp

will dump the configuration to a database named ``seiscomp``
(\ `PostgreSQL <https://www.postgresql.org/>`_ backend) on ``localhost``. For further
information, please refer to
the :external:ref:`bindings2cfg` documentation.

