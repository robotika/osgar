# Multi-Log Feature Implementation Handoff

This document provides a detailed step-by-step technical blueprint and codebase handoff for implementing the Multi-Log feature in OSGAR.

---

## 1. Architectural Checklist & Concepts

### A. Configuration Format
The input is a JSON file mapping nicknames to objects with `"file"` and an optional `"offset_sec"`:
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

### B. Timeline Reference
For each log $i$ under nickname $nick_i$:
- $start\_time_i$ is read from the log header.
- $offset\_sec_i$ defaults to `0.0`.
- The reference starting point is:
  $$T_{\text{ref}} = \min_{i} (start\_time_i + offset\_sec_i)$$
- A local packet timestamp $t_{\text{local}}$ is transformed to global time $t_{\text{global}}$ via:
  $$t_{\text{global}} = (start\_time_i + offset\_sec_i - T_{\text{ref}}) + t_{\text{local}}$$

### C. Stream Naming & Virtual IDs
- Stream names are prefixed: `"{nickname}.{original_stream_name}"` (e.g., `"pat.platform.pose2d"`).
- Virtual stream IDs are assigned sequentially starting from `1` matching the index $+ 1$ of the stream in the combined flat list.
- Keep a bidirectional map or structured lookup for nickname + local stream ID $\longleftrightarrow$ Virtual Stream ID.

### D. Handling of Stream 0 (System Stream)
- Stream ID `0` contains local metadata (configurations and registered stream names).
- Since metadata is parsed upfront during initialization, the Multi-Log reader generator **MUST explicitly filter out and skip all local stream ID 0 packets** during the merging step. This prevents multiple competing system packets from polluting the merged data stream.

---

## 2. Implementation Plan

### Step 1: Support Multi-Log in `osgar/logger.py` Helpers
We must extend the standard helper functions to transparently detect a Multi-Log configuration (typically by checking if the filename is a dictionary, or ends with `.json`).

#### A. `lookup_stream_names(filename)`
```python
def lookup_stream_names(filename):
    if isinstance(filename, dict) or filename.endswith('.json'):
        # 1. Load config if it is a JSON path
        # 2. For each nickname:
        #    a. Get local stream names using standard lookup_stream_names
        #    b. Prefix each stream: f"{nickname}.{local_name}"
        # 3. Return the consolidated flat list of prefixed names
        pass
```

#### B. `lookup_stream_id(filename, stream_name)`
```python
def lookup_stream_id(filename, stream_name):
    if isinstance(filename, dict) or filename.endswith('.json'):
        # 1. Resolve flat stream names list via lookup_stream_names(filename)
        # 2. Return stream_names.index(stream_name) + 1
        pass
```

#### C. `lookup_config(filename)`
```python
def lookup_config(filename):
    if isinstance(filename, dict) or filename.endswith('.json'):
        # Return a dictionary where keys are nicknames and values are local log configurations
        # e.g., { nickname: lookup_config(file) }
        pass
```

---

### Step 2: Implement the Low-Level `MultiLogReader`

Create a clean class `MultiLogReader` (either as a standalone class or subclassing `LogReader` to replicate its iterator/context manager API).

#### Core Responsibilities:
1. **Initialize and Parse Sub-Logs**:
   - Load the JSON/dictionary configuration.
   - Instantiate a standard `LogReader` for each source file.
   - Collect each reader's `start_time`, compute $T_{\text{ref}}$, and compile the map of Virtual IDs.
2. **Handle filtering (`only_stream_id`)**:
   - If `only_stream_id` is passed, map those Virtual IDs back to their local log sources and local stream IDs.
   - Configure each sub-`LogReader` with its corresponding local stream ID filter. This keeps file-parsing extremely fast and lightweight!
3. **Chronological Stream Merging (`heapq.merge`)**:
   - Convert the generator of each individual sub-log to output adjusted global timestamps and virtual stream IDs:
     ```python
     def sub_generator(nickname, reader, local_to_virtual_map, delta_offset):
         for dt, local_id, data in reader._read_gen():
             global_dt = dt + delta_offset
             virtual_id = local_to_virtual_map[local_id]
             yield global_dt, virtual_id, data
     ```
   - Feed these individual generators into `heapq.merge(..., key=lambda packet: x[0])`.

---

### Step 3: Extend `LogReaderEx` (The Primary User API)

Update `LogReaderEx` to cleanly support Multi-Logs, providing automatic deserialization and friendly string-based names.

```python
class LogReaderEx:
    def __init__(self, filename, names=None):
        if isinstance(filename, dict) or filename.endswith('.json'):
            # Multi-Log mode
            self.stream_names = lookup_stream_names(filename)
            # Map requested names to virtual stream IDs
            only_stream_id = None
            if names is not None:
                only_stream_id = [self.stream_names.index(name) + 1 for name in names]
            self.reader = MultiLogReader(filename, only_stream_id=only_stream_id)
        else:
            # Standard single-log mode
            ...
```

For its generator:
```python
def _read_gen(self):
    if self.is_multi_log:
        for dt, virtual_id, raw_data in self.reader:
            # Yield (dt, prefixed_name, deserialized_data)
            stream_name = self.stream_names[virtual_id - 1]
            yield dt, stream_name, deserialize(raw_data)
```

---

## 3. Recommended Test Suite

We should write automated unit tests in `osgar/test_multilog.py` (or as a new test case class inside `osgar/test_logger.py`).

### Verification Checklist:
1. **Mock Data Creation**:
   - Generate two small temporary OSGAR log files with known `start_time` values (e.g. 10 seconds apart) and some dummy messages.
   - Create a corresponding Multi-Log configuration JSON.
2. **Metadata Verification**:
   - Assert `lookup_stream_names` correctly prefixes and flattens all streams.
   - Assert `lookup_config` correctly aggregates individual configurations.
3. **Unified Time Sorting & Merging**:
   - Assert that reading via `LogReaderEx` on the multi-log JSON returns packets in perfect global chronological order.
   - Confirm timestamps are correctly offset based on the earliest start time.
4. **Filtering**:
   - Confirm filtering by stream names (e.g., `names=["pat.platform.pose2d"]`) correctly limits output and avoids processing unneeded sub-logs.
