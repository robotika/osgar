# Graceful shutdown in the threaded recorder

`bus.request_stop()` requests shutdown of the entire run. It wakes the recorder
without closing any module's communication. `bus.shutdown()` remains local to
one handle. SIGINT, application completion (the legacy `app` behavior), and
`--duration` use the same teardown sequence.

The recorder moves through these states:

1. `RUNNING`: normal operation.
2. `STOPPING`: call controller `request_stop()` methods and wait for their threads
   to finish. Transports and the logger remain available for final commands.
3. `DRAINING`: call transport `request_stop()` methods and wait for their threads
   to finish. Queue-based transports can process commands queued before their
   shutdown sentinel.
4. `CLOSED`: seal bus handles against late publications and close the log.

Each module's stop method and join run in a dedicated daemon worker. All modules
in a group receive their stop requests concurrently. Returning from `join()` is
the completion signal; it must mean all of that module's threads have finished.
The recorder waits for the whole controller group before stopping any transport.
No ordering within a group is promised.

## Configuration

Modules default to `"shutdown_group": "controller"`. Mark the low-level
transports that must remain available during controller cleanup explicitly:

```json
{
  "version": 2,
  "robot": {
    "shutdown_timeout": 5.0,
    "modules": {
      "controller": {
        "driver": "myrobot.controller:Controller"
      },
      "serial": {
        "driver": "serial",
        "shutdown_group": "transport",
        "init": {"port": "/dev/ttyUSB0", "speed": 115200}
      }
    },
    "links": [["controller.raw", "serial.raw"]]
  }
}
```

`shutdown_timeout` is a positive number of seconds, defaulting to 5, for each
group as a whole. `--duration 10` begins graceful shutdown after 10 seconds;
cleanup may take up to two additional group timeouts. A global request can
begin shutdown earlier. Timeout or cleanup failure is logged, and teardown
continues to the next group. This bounds waiting for stop methods and joins;
Python cannot forcibly terminate a stuck thread or interrupt arbitrary device
I/O. Drivers still need finite I/O timeouts. A surviving non-daemon thread can
keep the process alive. Synchronous slot handlers must also return promptly:
they execute while the logger lock is held and can delay final log closure.

## Sending a final command

For nodes using the default `Node.run()`, override `on_shutdown()`:

```python
class Controller(Node):
    def on_shutdown(self):
        self.publish('raw', b'STOP\n')
```

This hook runs in the node thread when its processing loop exits, including
after an error. It runs before the node's join completes. The node's own local
handle has been shut down, but publishing remains available until final bus
closure. Nodes overriding `run()` must arrange their own cleanup, for example
in a `finally` block. Existing custom `request_stop()` cleanup is also supported.

TCP, serial, and PeakCAN output loops use `listen()` until the shutdown sentinel,
so commands already queued ahead of it are processed before output exits.
Transports with different loop behavior need their own drain implementation.
For a CAN-over-serial bridge, leave the protocol bridge in the controller group
and put the underlying serial driver in the transport group, so the bridge's
final CAN instruction can still be forwarded.

This provides ordering and thread completion, not device acknowledgement or
guaranteed delivery. A disconnected device, partial write, or timeout can still
lose a command. Waiting for device acknowledgements and transport-specific flush
operations is a separate step. After a controller times out, late commands can
also miss the transport's shutdown sentinel; that is a failed graceful shutdown.

These phases apply to `osgar.record`, not the separate ZeroMQ router backend.
