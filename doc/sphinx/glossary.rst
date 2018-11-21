Glossary
========

.. glossary::

   bus
      The communication backbone, providing `publish` and `listen` operations.
      The bus takes care of sequencing all messages within the sysem and
      serialization via logging facility.

   link
      Directed connection of two nodes. The source node generates messages by
      `publish` call and the destinaion node accepts them via `listen` call.

   logfile
      Binary file containing message data with their source and timestamp.

   node
      The basic processing unit attached to `bus`. It has fixed set of inputs
      and outputs. Some nodes implement communication with outer world like TCP.

   OSGAR
      Open Source Garden/Generic Autonomous Robot (Python library)

   robot
      The collection of interacting `nodes` via `bus`. This typically corresponds
      to a machine with set of sensors and efectors in the real world.

   stream
      Output from node without specified destination like for `link`.    

   timestamp
      Python `datetime.timedelta` with microsecond precision starting from zero
      with the beginning of the program.

