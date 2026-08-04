# Multi-Log Feature Design

This document details the design and implementation strategy for the **Multi-Log** feature in OSGAR. 
The goal of this feature is to allow users to process and replay multiple independent, temporally overlapping log files under a unified interface, as if they were reading from a single "virtual" log file.

---

## 1. Use Cases

In real-world operations, data often comes from multiple independent sources:
1. **Multi-Robot Operations**: Coordinating or analyzing the behavior of multiple autonomous robots (e.g., `pat` and `m03`).
2. **Robot with External Sensors**: A robot logging its internal states (pose, lidar) while an external stationary camera or motion capture system logs ground-truth data in a separate file.
3. **Algorithmic Reprocessing**: An old recording of raw sensor data replayed/reprocessed into a new log with updated pose estimations. We want to read the raw streams from the old log and the processed streams from the new log simultaneously.

To analyze these systems holistically, we need a unified time domain where streams from different logs can be accessed synchronously.

---

## 2. Configuration Schema

A Multi-Log session is defined via a JSON configuration. Each nickname maps to an object specifying the file path and an optional time offset (in seconds) to align log timelines. 

If `offset_sec` is omitted, it defaults to `0.0`, which is ideal for systems that are already temporally synchronized (e.g., when multiple sensors or robots record on the same machine/network using synchronized clocks).

```json
{
  "m03": {
    "file": "m03-matty-on-pat-redroad-260801_105050.log",
    "offset_sec": -1.23
  },
  "pat": {
    "file": "pat-dh26-260801_105022.log"
  }
}
```

---

## 3. Core Concepts

### A. Prefixed Stream Names
To prevent name collisions between overlapping streams (e.g., both logs having a `platform.pose2d` stream), all streams from a source are prefixed with their nickname.
- Stream `platform.pose2d` from `pat` becomes `pat.platform.pose2d`.
- Stream `oak.color` from `m03` becomes `m03.oak.color`.

### B. String-Based Stream Names (Primary User-Facing Interface)
While integer stream IDs are used internally for backwards compatibility, the primary and recommended way for users and post-processing tools to identify and consume streams is via their **string-based names** (e.g., `"pat.platform.pose2d"`, `"m03.oak.color"`). String names are far more obvious and intuitive when combining multiple log sources.

For internal mapping and compatibility with low-level readers, the Multi-Log reader assigns sequential **Virtual Stream IDs** starting from `1` across all logs.

Suppose:
- `pat` has 2 streams: `["platform.raw", "platform.pose2d"]`
- `m03` has 2 streams: `["oak.raw", "oak.color"]`

The reader maps them as follows:

| Virtual ID | Full Prefixed Name (with two dots) | Source Log | Source Local Stream ID |
|------------|------------------------------------|------------|------------------------|
| `1`        | `pat.platform.raw`                 | `pat`      | `1`                    |
| `2`        | `pat.platform.pose2d`              | `pat`      | `2`                    |
| `3`        | `m03.oak.raw`                      | `m03`      | `1`                    |
| `4`        | `m03.oak.color`                    | `m03`      | `2`                    |

Calling `lookup_stream_names()` on a Multi-Log configuration returns:
`['pat.platform.raw', 'pat.platform.pose2d', 'm03.oak.raw', 'm03.oak.color']`.

### C. Handling of Stream 0 (System/Info Stream)
Stream ID `0` in standard OSGAR logs is a special system stream containing metadata such as stream names and driver configurations. 

When merging multiple logs:
1. **Upfront Metadata Retrieval**: The global configuration, stream names, and local settings are retrieved during the initialization of `MultiLogReader` (or via extended helper functions like `lookup_stream_names` and `lookup_config`).
2. **Filtering During Stream Iteration**: During chronological packet merging, `MultiLogReader` **explicitly filters out and skips all local stream ID `0` packets**. This avoids polluting the unified stream with multiple conflicting system configuration packets, which could break tools expecting a single metadata block.

---

## 4. Time Synchronization & Merging

Each individual log contains its own absolute UTC `start_time` in its header, and relative `timedelta` offsets for subsequent packets.

### A. Global Timeline Reference
We define the global reference start time $T_{\text{ref}}$ as the absolute earliest start time among all configured logs:

$$T_{\text{ref}} = \min_{i} (start\_time_i + offset\_sec_i)$$

For any log $i$, a packet with a local relative timestamp $t_{\text{local}}$ is mapped to the global timeline as:

$$t_{\text{global}} = (start\_time_i + offset\_sec_i - T_{\text{ref}}) + t_{\text{local}}$$

Since $T_{\text{ref}}$ is the absolute minimum, $t_{\text{global}}$ is guaranteed to be a positive `timedelta` (assuming no negative local timestamps).

### B. Efficient K-Way Stream Merging
To yield packets chronologically, we perform a $k$-way merge of the generators of each log file. Since each file's packets are already written in chronological order, we can use Python's `heapq.merge` for highly efficient, constant-memory streaming:

```python
import heapq

# Concept implementation of merging
def merged_generator(readers):
    generators = []
    for nick, reader in readers.items():
        generators.append(
            (global_time(dt, nick), nick, stream_name, deserialized_data)
            for dt, stream_name, deserialized_data in reader
        )
    for dt_g, nick, stream_name, deserialized_data in heapq.merge(*generators, key=lambda x: x[0]):
        # Yield absolute timeline packet
        yield dt_g, f"{nick}.{stream_name}", deserialized_data
```

---

## 5. API Design & Proposed Classes

We introduce and support two main levels of log reading, with a strong focus on `LogReaderEx`.

### A. `LogReaderEx` (Primary User-Facing Interface)
`LogReaderEx` is the primary utility we support. It is the most user-friendly reader because it automatically resolves stream names and deserializes the payload.

In the first round, `LogReaderEx` will be extended to transparently handle Multi-Log configurations. When initialized with a Multi-Log JSON file, it will instantiate a multi-log session and yield fully deserialized stream data identified by their intuitive string-based names:

```python
# Usage Example:
with LogReaderEx("multi_config.json") as log:
    for dt, stream_name, data in log:
        # stream_name will be e.g. "pat.platform.pose2d"
        # data will be fully deserialized
        print(dt, stream_name, data)
```

We can also filter streams by name during initialization:
```python
with LogReaderEx("multi_config.json", names=["pat.platform.pose2d", "m03.oak.color"]) as log:
    for dt, stream_name, data in log:
        ...
```

### B. `MultiLogReader` (Underlying Low-Level Reader)
A subclass or drop-in wrapper of `LogReader`. It works with raw bytes and Virtual Stream IDs for low-level compatibility.

```python
class MultiLogReader:
    def __init__(self, config_file_or_dict, only_stream_id=None, clip_start_time_sec=0.0, clip_end_time_sec=None):
        # 1. Load configuration (dictionary or JSON path)
        # 2. Open LogReader instances for each file
        # 3. Calculate T_ref and construct virtual stream map
        # 4. Filter local readers if only_stream_id is provided
        pass

    def __iter__(self):
        # Yields (global_dt, virtual_stream_id, data_bytes)
        pass
```

### C. Transparent Helper Functions
We will extend existing functions in `osgar/logger.py` to seamlessly detect Multi-Log configurations:

- **`lookup_stream_names(filename)`**:
  If `filename` is a Multi-Log JSON, it loads the config, fetches names for each log, prefixes them with nicknames, and returns the flat combined list of prefixed names.
- **`lookup_stream_id(filename, stream_name)`**:
  If the target is a Multi-Log, it translates a prefixed name (e.g., `pat.platform.pose2d`) to its **Virtual Stream ID**.
- **`lookup_config(filename)`**:
  Returns a merged dictionary of configurations nested by nicknames (e.g., `{"pat": pat_config, "m03": m03_config}`).

---

## 6. Open Points & Future Enhancements

1. **How should we handle non-overlapping logs?**
   - *Current thought*: Because we do not know the end of a logfile in advance (especially with growing logfiles read with `follow=True`), we cannot reliably determine time span overlap at startup. For the initial version, we will ignore time-span overlap checking and simply merge whatever packets are available sequentially. Complex stitching and alignment of disjoint log spans will be deferred to future releases.

2. **LogIndexedReader Compatibility**
   - *Current thought*: `LogIndexedReader` is highly useful for visualization and analysis (e.g., in `osgar.tools.lidarview`). However, there is already an inherent challenge in handling backward steps with H.264 or H.265 encoded video streams due to keyframe/inter-frame dependencies. For the Multi-Log feature, we will focus on simplified forward-only stepping. If building a unified merged index and supporting general random-access becomes too complex, the implementation of `MultiLogIndexedReader` will be postponed to a subsequent iteration.

3. **Time Synchronization Calibration**
   - *Current thought*: Hand-tuning `offset_sec` is tedious. While we could eventually support an automatic time calibration tool or a configuration property that aligns logs based on matching events or cross-correlation of signals (such as aligning GPS logs, or detecting a physical flash/bump visible in multiple logs), we will postpone any automatic alignment tools to a future iteration. The first version will rely strictly on manual `offset_sec` definitions in the configuration.
