Architecture
============


.. image:: media/sc-system-scdetect.svg
  :width: 500
  :align: center


Above, the modular organization of SeisComP
with :external:ref:`messaging system <concepts_messaging>` (mediator),
:external:term:`RecordStream`
interface (waveform server) and
:external:ref:`database <concepts_database>`
including ``scdetect-cc``\ 's role in the SeisComP overall architecture. In SeisComP
language ``scdetect-cc`` is implemented in accordance to a :external:term:`trunk module <trunk>`.

From an architectural point of view ``scdetect-cc`` is positioned somewhere
between :external:ref:`scautopick` and :external:ref:`scautoloc`. That
is, ``scdetect-cc`` fetches waveform data by means of
the :external:term:`RecordStream`
interface, but it also uses data products (i.e. EventParameters) for template
generation. If connected to the :external:ref:`messaging system
<concepts_messaging>`, results (i.e. declared origins (including both station
magnitudes and network magnitudes), picks and amplitudes) are sent to the
:external:ref:`messaging system <concepts_messaging>`.

For further information with regard to the SeisComP architecture please refer to
the :external:ref:`SeisComP documentation <overview>`.
