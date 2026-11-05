Drivers
=======

The "drivers" directory contains implementation of all drivers for various
devices. Sensors, effectors as well as the mobile platforms are included.
The driver takes care of a single device, or better to say a single
communication stream (serial.Serial for example). The data are collected,
logged and parsed for further processing.

Logging
-------

There is at least one stream logged per driver - the input for sensors and
the output for effectors. If bidirectional communication is required then
both streams are logged. The "stream ID" is part of the driver configuration.

Use cases
---------

There are two basic use cases: real run and replay. In real run real port
is opened and dedicated Thread::run() takes care of reading data with timeout
and the processing.

In the "replay" mode the module receives bus data from a log file.
This mode is mainly designed for debugging,
development of parsers and also verification of identical outputs on
different platforms (robot PC vs. notebook for example).

Replay does not automatically replace device connections opened by a driver.
A driver that connects to an external server in its startup code still needs
that server, or an explicit mock or replay implementation.

Drivers using async libraries
----------------------------

Inherit from ``osgar.asyncnode.AsyncNode`` when using a device library with an
``asyncio`` API. Each instance runs an event loop in its own node thread.
Override ``main()`` to open the connection, then await ``listen_loop()`` to
process bus messages. For example, a TCP client can use:

.. code-block:: python

   import asyncio
   from osgar.asyncnode import AsyncNode


   class AsyncTCPWriter(AsyncNode):
       def __init__(self, config, bus):
           super().__init__(config, bus)
           self.host = config['host']
           self.port = config['port']

       async def main(self):
           reader, self.writer = await asyncio.open_connection(self.host, self.port)
           try:
               await self.listen_loop()
           finally:
               self.writer.close()
               await self.writer.wait_closed()

       async def on_raw(self, data):
           self.writer.write(data)
           await self.writer.drain()

Handlers can be synchronous or async. The listen loop finishes each handler
before dispatching the next message; awaiting I/O allows other event-loop
tasks to run but does not dispatch another bus message. Avoid blocking calls
inside handlers. Use ``await self.sleep_async(seconds)`` for an elapsed real-time
delay, and ``self.time`` for the timestamp of the current bus message.

Register outputs in ``__init__`` as usual and call ``self.publish()`` without
``await``. Configuration and links use the same format as synchronous drivers.
During a live run, ``request_stop()`` queues a shutdown sentinel without
cancelling ``main()``. The bus loop drains earlier messages and returns;
``main()`` must then finish its connection cleanup, including awaited context
managers or ``finally`` blocks. Background tasks must also finish or be cancelled
by the driver. After a shutdown timeout, ``force_stop()`` cancels blocked async
work; allow that cancellation to propagate. Low-level async communication drivers
should use the ``transport`` shutdown group so controllers can send final commands.

The `AsyncEcho example <https://github.com/robotika/osgar/tree/master/examples/asyncecho>`_
demonstrates a TCP server and an async application communicating over the bus.

