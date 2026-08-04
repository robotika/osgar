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

A Multi-Log session is defined via a JSON configuration. The simplest form maps a nickname to a log file path:

```json
{
  "m03": "m03-matty-on-pat-redroad-260801_105050.log",
  "pat": "pat-dh26-260801_105022.log"
}
```

To support advanced options (like manual clock synchronization offsets), the configuration also supports a detailed object-based structure per nickname:

```json
{
  "m03": {
    "file": "m03-matty-on-pat-redroad-260801_105050.log",
    "offset_sec": -1.23
  },
  "pat": {
    "file": "pat-dh26-260801_105022.log",
    "offset_sec": 0.0
  }
}
```

---

## 3. Core Concepts

### A. Prefixed Stream Names
To prevent name collisions between overlapping streams (e.g., both logs having a `platform.pose2d` stream), all streams from a source are prefixed with their nickname.
- Stream `platform.pose2d` from `pat` becomes `pat.platform.pose2d`.
- Stream `oak.color` from `m03` becomes `m03.oak.color`.

### B. Virtual Stream IDs
To maintain strict compatibility with OSGAR's internal mechanics and existing post-processing scripts, the Multi-Log reader assigns sequential **Virtual Stream IDs** starting from `1` across all logs.

Suppose:
- `pat` has 2 streams: `["raw", "pose2d"]`
- `m03` has 2 streams: `["raw", "color"]`

The reader maps them to virtual stream IDs as follows:

| Virtual ID | Full Prefixed Name | Source Log | Source Local Stream ID |
|------------|--------------------|------------|------------------------|
| `1`        | `pat.raw`          | `pat`      | `1`                    |
| `2`        | `pat.pose2d`       | `pat`      | `2`                    |
| `3`        | `m03.raw`          | `m03`      | `1`                    |
| `4`        | `m03.color`        | `m03`      | `2`                    |

Calling `lookup_stream_names()` on a Multi-Log configuration returns `['pat.raw', 'pat.pose2d', 'm03.raw', 'm03.color']`.

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
            (global_time(dt, nick), nick, stream_id, data)
            for dt, stream_id, data in reader
        )
    for dt_g, nick, local_id, data in heapq.merge(*generators, key=lambda x: x[0]):
        virtual_id = to_virtual_id(nick, local_id)
        yield dt_g, virtual_id, data
```

---

## 5. API Design & Proposed Classes

We introduce a new module/classes to handle Multi-Log reading transparently.

### A. `MultiLogReader` (Subclass or drop-in wrapper of `LogReader`)
This class provides the exact same iterator interface as `LogReader`, making it seamless for existing tools.

```python
class MultiLogReader:
    def __init__(self, config_file_or_dict, only_stream_id=None, clip_start_time_sec=0.0, clip_end_time_sec=None):
        # 1. Load configuration (dictionary or JSON path)
        # 2. Open LogReader instances for each file
        # 3. Calculate T_ref and construct virtual stream map
        # 4. If only_stream_id is provided, map virtual IDs back to local IDs and filter local readers
        pass

    def __iter__(self):
        # Yields (global_dt, virtual_stream_id, data)
        pass
```

### B. Transparent Helper Functions
We will extend existing functions in `osgar/logger.py` to seamlessly detect Multi-Log configurations (e.g., if the passed path ends with `.json` or is recognized as a JSON file):

- **`lookup_stream_names(filename)`**:
  If `filename` is a Multi-Log JSON, it loads the config, fetches names for each log, prefixes them with nicknames, and returns the flat combined list of prefixed names.
- **`lookup_stream_id(filename, stream_name)`**:
  If the target is a Multi-Log, it translates a prefixed name (e.g. `pat.platform.pose2d`) to its **Virtual Stream ID**.
- **`lookup_config(filename)`**:
  Returns a merged dictionary of configurations nested by nicknames (e.g., `{"pat": pat_config, "m03": m03_config}`).

---

## 6. Open Points & Future Enhancements

1. **How should we handle non-overlapping logs?**
   - *Current thought*: Raise a warning during initialization if the logs have absolutely no overlap, but still allow processing if the user explicitly wants to stitch consecutive logs.

2. **LogIndexedReader Compatibility**
   - For rapid indexing and random access via `__getitem__`, `LogIndexedReader` uses `mmap` and building an index.
   - For Multi-Log, we could build a combined virtual index: a list of `(pos_in_sub_log, global_dt, nickname, local_stream_id)`.
   - Should we implement `MultiLogIndexedReader` if random access is required by UI/analysis tools?

3. **Time Synchronization Calibration**
   - Hand-tuning `offset_sec` is tedious.
   - We could support an automatic time calibration tool or a configuration property that aligns logs based on matching events or cross-correlation of signals (e.g. aligning GPS logs, or detecting a physical flash/bump visible in multiple logs).

4. **Integration with `LogReaderEx`**
   - `LogReaderEx` is highly used as it automatically deserializes data. `MultiLogReader` should be fully compatible with `LogReaderEx` so that calling `LogReaderEx` on a multi-log JSON yields `(global_dt, "nickname.stream_name", deserialized_data)`.
