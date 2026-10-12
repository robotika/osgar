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

Start OSGAR with the example configuration:

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
