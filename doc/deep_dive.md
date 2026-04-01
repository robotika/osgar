# Deep Dive for OSGAR Developers

This document explains the internal workings of the OSGAR system, specifically focusing on how modules are initialized, how they communicate, and how data is recorded.

## 1. System Starting Modules (`osgar.record`)

The entry point for recording data in OSGAR is `osgar.record`. When you run `python -m osgar.record config.json`, the following sequence occurs:

1.  **Configuration Loading**: The system loads the JSON configuration file using `osgar.lib.config.config_load`. This merges any included configurations and handles parameter overrides.
2.  **LogWriter Initialization**: A `LogWriter` is created to handle the output log file. The entire configuration is serialized and written as the first record (stream 0) in the log.
3.  **Recorder Creation**: The `Recorder` class is instantiated with the configuration and the logger.
4.  **Bus and Handlers**: The `Recorder` creates a central `Bus` object. For each module defined in the configuration, a `_BusHandler` is created. This handler is the module's interface to the rest of the system.
5.  **Module Instantiation**: For each module, the driver class is located (using `get_class_by_name`) and instantiated. The module is passed its specific `init` configuration and its dedicated `bus` handle.
6.  **Connecting Modules**: After all modules are created, the `Bus.connect` method is called for each link in the configuration to establish communication paths.
7.  **Starting Threads**: Finally, `module.start()` is called for each module, which, for `Node`-based modules, starts a new Python thread.

## 2. Module Parameters in Config JSON

The configuration file defines the structure of the OSGAR application. Each module entry typically contains:

-   `driver`: The Python class name or alias (e.g., `osgar.drivers.gps:GPS`).
-   `init`: A dictionary of parameters passed to the module's `__init__` method.
-   `bus`: (Implicitly handled) Mapping of inputs and outputs.

Example:
```json
"modules": {
  "gps": {
    "driver": "osgar.drivers.gps:GPS",
    "init": {
      "port": "COM3",
      "baudrate": 4800
    }
  }
}
```

## 3. Python Threads and Communication

OSGAR's standard implementation uses Python's `threading.Thread`. Each module runs in its own thread, allowing for concurrent execution.

-   **Standard Version**: Modules inherit from `osgar.node.Node`, which is a `threading.Thread`. They spend most of their time in a `listen()` loop, waiting for data on their input queue.
-   **ZMQ Option**: While threads are the default, OSGAR also supports an alternative architecture using processes and communication via a **ZMQ router** (`osgar/zmqrouter.py`). This is useful for multi-language support or better process isolation, but the core principles of the bus remain the same.

## 4. Module Linking and I/O Names

Links in the configuration define how data flows between modules:

```json
"links": [
  ["gps.position", "app.position"],
  ["app.desired_speed", "base.speed"]
]
```

-   The first part (e.g., `gps.position`) is `sender.output_channel`.
-   The second part (e.g., `app.position`) is `receiver.input_channel`.
-   When a module calls `self.publish('position', data)`, the `_BusHandler` identifies all connected receivers and puts the data into their respective input queues.

## 5. Storage in the Logfile

Everything that passes through the bus is recorded in the logfile via `LogWriter`.

-   **Stream Registration**: Each output channel (e.g., `gps.position`) is registered as a unique "stream" with an ID.
-   **Record Format**: Each log entry consists of:
    -   `stream_id`: Identifying the source.
    -   `timestamp`: The time the data was published.
    -   `data`: Serialized (and optionally compressed) payload.
-   **Replayability**: Because the configuration and all bus messages are logged, the exact state and behavior of the system can be reproduced by "replaying" the log.

## 6. Input Queue Implementation

Each module's `_BusHandler` contains a `queue.Queue()` (a thread-safe FIFO queue).

1.  **Publishing**: When `publish(channel, data)` is called:
    -   The data is serialized and written to the log.
    -   The data (wrapped with its timestamp and target input channel name) is `put()` into the `queue` of every connected module.
2.  **Listening**: When a module calls `listen()` (or `update()`):
    -   It calls `self.queue.get()`, which blocks until data is available.
    -   The module then processes the message, typically by calling a handler method (e.g., `on_position`).

This architecture ensures that modules are decoupled and that data is processed in the order it was received, with the system log serving as a perfect record of all interactions.
