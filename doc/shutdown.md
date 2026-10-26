# Graceful shutdown in the threaded recorder

`bus.request_stop()` requests shutdown of the entire run. It wakes the recorder
without closing any module's communication. `bus.shutdown()` remains local to
one handle. SIGINT, application completion (the legacy `app` behavior), and
`--duration` use the same teardown sequence.

The recorder moves through these states:

1. `RUNNING`: normal operation.
2. `STOPPING`: prepare controllers, wait for their readiness, then close them.
   Transports and the logger remain available for commands and responses.
3. `DRAINING`: prepare and close transports. Queue-based transports can process
   commands queued before their shutdown sentinel.
4. `CLOSED`: seal bus handles against late publications and close the log.

Each module's shutdown coordination and join run in a dedicated daemon worker.
Modules in a group prepare concurrently. A module reporting readiness receives
`close`; the recorder waits for its join to finish. The recorder waits for the
whole controller group before preparing any transport. No ordering within a
group is promised.

Existing modules default to `"shutdown_protocol": "legacy"`: the recorder calls
their existing `request_stop()` and `join()` methods without lifecycle messages.
Returning from `join()` must mean all of that module's threads have finished.
This fallback preserves existing drivers while they adopt the message protocol.

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
        "driver": "myrobot.controller:Controller",
        "shutdown_protocol": "messages"
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
cleanup may take up to two additional group timeouts, plus a shared one-second
forced-cancellation allowance for each group that fails or times out. A global request can
begin shutdown earlier. Timeout or cleanup failure is logged, and teardown
continues to the next group. This bounds waiting for stop methods and joins;
Python cannot forcibly terminate a stuck thread or interrupt arbitrary device
I/O. Drivers still need finite I/O timeouts. A surviving non-daemon thread can
keep the process alive. Synchronous slot handlers must also return promptly:
they execute while the logger lock is held and can delay final log closure.

After a timeout or cleanup failure, the recorder shuts down the local handle
and calls the module's optional `force_stop()` in a bounded daemon worker.
`AsyncNode.force_stop()` cancels blocked async work. The cancellation allowance
is shared by all failed modules in the group. Modules must allow cancellation
to propagate; a coroutine that suppresses it indefinitely still cannot be
forcibly terminated by Python.

## Lifecycle messages and final commands

Opt in with `"shutdown_protocol": "messages"` on the module configuration.
The framework provides a reserved `shutdown` input, handled by
`on_shutdown(self, data)` just like any other input. No configuration links are
required or permitted for that input when opted in. `_system` is a reserved
module name.

Messages contain:

```python
{'phase': 'prepare', 'reason': 'duration', 'timeout': 5.0, 'deadline': 15.0}
```

`deadline` is in seconds relative to the log's start, on the same timeline as
`self.time.total_seconds()`. Both phases share the group's deadline; `timeout`
is the remaining group budget at publication. The recorder enforces its timeout
using a monotonic clock. Reasons are `requested`, `duration`, `sigint`, or
`exception`. In replay, compare deadlines against recorded bus time rather than
the current wall clock.

For a controller without device acknowledgements:

```python
class Controller(Node):
    def on_shutdown(self, data):
        if data['phase'] == 'prepare':
            self.stopping = True  # normal handlers must stop generating commands
            self.publish('raw', b'STOP\n')
            self.bus.shutdown_ready()
        elif data['phase'] == 'close':
            super().on_shutdown(data)
```

The default `Node.on_shutdown(data)` reports immediate readiness on `prepare`
and calls local `request_stop()` on `close`. During prepare the processing loop,
`listen()`, and communication remain alive. Reporting readiness is explicit,
idempotent, and does not close the handle. The subsequent close message ends
the node normally. Do not block inside the handler waiting for a response:
return so that ordinary input handlers can process it.

`AsyncNode` dispatches both ordinary and async `on_shutdown(self, data)` handlers.
Its normal `request_stop()` queues the local shutdown sentinel without cancelling
`main()`. The bus loop drains earlier messages, then returns so async context
managers and `finally` blocks can finish. Only `force_stop()` requests task
cancellation. Custom `main()` implementations must finish when `listen_loop()`
returns, rather than wait forever for an unrelated background task. Await or
cancel those background tasks during cleanup. Place low-level async transports
in the transport shutdown group, just as for synchronous TCP or serial drivers.

This replaces the earlier zero-argument `on_shutdown()` cleanup hook. Local
shutdown or an unexpected thread exit does not call the handler automatically.
Nodes overriding `run()` must dispatch the lifecycle input themselves or retain
legacy stop-and-join behavior. An opted-in node that has already exited cannot
report readiness and will consume the group's timeout.

The system logs messages as `_system.shutdown_<module>` and readiness as
`<module>._shutdown_ready`. These stream names are reserved. Replay automatically
maps the system stream to the module's `shutdown` input, and strict replay checks
readiness along with other outputs. Forced replay also supports the protocol.
During replay, `bus.request_stop()` does not terminate the node: it continues
through the recorded lifecycle inputs. Local `bus.shutdown()` still exits
immediately. Logs predating the lifecycle protocol contain no shutdown inputs
to replay.

## Waiting for device acknowledgements

On prepare, send the final command but defer `shutdown_ready()` until the normal
device input handler recognizes the matching acknowledgement:

```python
def on_shutdown(self, data):
    if data['phase'] == 'prepare':
        self.stopping = True
        self.publish('raw', make_stop_command())
    else:
        super().on_shutdown(data)

def on_raw(self, data):
    # Parse/framing and correlation belong to the device protocol driver.
    if self.stopping and is_ack_for_pending_stop(data):
        self.bus.shutdown_ready()
```

Only signal readiness after all required cleanup is complete. Handle framing,
command identifiers, stale acknowledgements, and retransmission according to
the device's protocol. Repeated lifecycle callbacks represent phase changes,
not readiness polling. After reporting readiness, a controller must stop
generating device commands.

TCP, serial, and PeakCAN output loops use `listen()` until the shutdown sentinel,
so commands already queued ahead of it are processed before output exits.
Transports with different loop behavior need their own drain implementation.
For a CAN-over-serial bridge, leave the protocol bridge in the controller group
and put the underlying serial driver in the transport group, so the bridge's
final CAN instruction can still be forwarded.

This protocol allows controllers to wait for acknowledgements; it does not
invent an acknowledgement for devices that have none. A transport write
completing does not prove that a device received or applied the instruction.
Transport-specific flushing and partial-write handling remain driver concerns.
No acknowledgement before the deadline is a failed graceful shutdown: the
recorder logs the timeout, forces local shutdown, and proceeds. Late commands
can miss the transport's shutdown sentinel after a timeout.

These phases apply to `osgar.record`, not the separate ZeroMQ router backend.
