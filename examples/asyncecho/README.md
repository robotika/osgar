AsyncEcho Example
=================

This example demonstrates how to use Python's `asyncio` inside OSGAR using the `AsyncNode` base class.

Components
----------

1. **AsyncEcho Driver (`osgar.drivers.asyncecho:AsyncEcho`)**:
   An asynchronous TCP server running an asyncio event loop inside its thread.
   It accepts TCP client connections, forwards data received over TCP to the OSGAR bus on `raw`, and broadcasts any data received on `raw` back to all connected TCP clients.

2. **EchoApp (`examples.asyncecho.asyncecho_app:EchoApp`)**:
   An application node inheriting from `AsyncNode` that receives `raw` data asynchronously (`async def on_raw`) and publishes a reply back to `raw`.

Running the Example
-------------------

From the repository root, start OSGAR with the example configuration:

```bash
python3 -m osgar.record examples/asyncecho/asyncecho.json
```

In another terminal, connect using `nc` or `telnet`:

```bash
nc 127.0.0.1 8888
```

Type a message and press Enter; you will receive the echoed response from `EchoApp` routed through the OSGAR bus:

```
hello
ECHO: hello
```

Stop the recording with Ctrl+C.

How AsyncNode Runs the Example
------------------------------

Each `AsyncNode` runs its own asyncio event loop in its node thread. `AsyncEcho`
overrides `main()` to start the TCP server and run the server alongside
`listen_loop()`. TCP callbacks publish received bytes on `echo.raw`.

The configuration links `echo.raw` to `app.raw`. `EchoApp` uses the default
`main()`, which runs `listen_loop()` and awaits its `on_raw()` handler for each
message. The handler builds the reply and calls `publish()` synchronously,
without `await`. The second link routes `app.raw` to `echo.raw`, where the
driver's async `on_raw()` broadcasts the reply to connected clients.

Bus handlers run sequentially within each node: the next message is dispatched
after the current handler finishes. Awaiting a network operation lets other
tasks on that event loop run, such as accepting a TCP connection, but does not
start processing the next bus message. TCP provides a byte stream, so one read
is not guaranteed to correspond to one line or one client write; this example
formats a reply for each received chunk.

Writing Your Own AsyncNode
--------------------------

Use `AsyncNode` for libraries with async APIs, such as an OPC UA client writing
bus data to a PLC. Override `main()` to establish the connection, then
`await self.listen_loop()` inside the connection's async context manager or a
`try`/`finally` block. This keeps setup, handlers, and cleanup on the same event
loop. Output registration and JSON links work as for a synchronous `Node`.

Use async I/O inside coroutines. Blocking calls such as `self.listen()`,
`self.sleep()`, or `time.sleep()` block the event loop. For a delay, use
`await self.sleep_async(seconds)`; this waits for elapsed real time.
`self.time` instead holds the bus timestamp of the message being dispatched.

During a live run, `request_stop()` shuts down the node's bus handle and
requests cancellation of `main()`. Let cancellation propagate and release
connections in context managers or `finally` blocks. In this example,
`AsyncEcho` closes the server and client connections, and the base class cancels
remaining tasks before closing the event loop.

Replay replaces bus input and output handling, but it does not mock network
connections. Replaying `EchoApp` consumes recorded bus messages without a TCP
client. Replaying the server driver still executes its server startup; likewise,
a custom OPC UA client's `main()` still needs its server or an explicit
replacement. AsyncNode does not automatically make external I/O replayable.

See the [developer deep dive](../../doc/deep_dive.md#asyncnode-and-async-io)
for the connection-lifecycle pattern and bus integration details.
