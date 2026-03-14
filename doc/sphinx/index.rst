.. osgar documentation master file, created by
   sphinx-quickstart on Sun Oct 28 10:25:10 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

OSGAR Documentation
===================

**Open Source Garden/Generic Autonomous Robot (Python library)**

OSGAR is a lightweight, multi-platform library for recording and replaying data from multiple `nodes` (modules such as sensors, robots, and applications) logged into a single file.

Key Features
------------

*   **Robust Logging:** All port data is logged with microsecond-resolution timestamps.
*   **Deterministic Replay:** Replaying recorded data produces identical outputs, enabling reliable debugging.
*   **Modular Architecture:** Robots are built as a set of independent modules connected via a communication "bus".
*   **Minimalistic & Multi-platform:** Runs on everything from high-end workstations to the Raspberry Pi Zero.

Applications & Robots
---------------------

OSGAR has been the core software framework for several prestigious competitions:

*   **DARPA Subterranean Challenge (SubT):** Coordinating heterogeneous robot fleets in underground environments.
*   **DARPA Triage Challenge (DTC):** Autonomous medical assessment using a fleet of **Matty** robots.
*   **Field Robot Event:** Autonomous agricultural robots in challenging outdoor environments.

.. image:: https://robotika.cz/competitions/roboorienteering/2016/jd-nav2.jpg
   :alt: John Deere X300R
   :width: 400px

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   tutorial
   logging
   eduro
   drivers
   rosproxy
   can
   msgtypes
   glossary

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
