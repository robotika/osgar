# OSGAR Message Bundle: Architectural Overview, Analysis & Implementation Handover

## 1. Executive Summary & Problem Analysis

### 1.1 The Synchronized Multi-Stream Dilemma
In robotics applications, sensor drivers and processing nodes frequently produce **multiple outputs that are intrinsically synchronized and available at the exact same physical moment** on the source side:
* **RGB-D Cameras** (e.g., RealSense, Luxonis OAK): Color frame + corresponding Depth map + Infrared.
* **Camera with Orientation**: Video frame + Camera heading / IMU orientation quaternion.
* **6-DOF IMU**: 3-axis Accelerometer + 3-axis Gyroscope.
* **Synchronized Encoders & Joints**: Joint angles + odometry ticks + bumper states.

### 1.2 Current OSGAR Implementation & Pain Points
OSGAR’s architecture relies on a log-centric, hub-and-spoke message bus (`osgar.bus.Bus`, `osgar.node.Node`, `osgar.logger.LogWriter`). Today, communication is strictly channel-by-channel:

```
Source Node                         Bus & Logger                       Receivers
------------------------------------------------------------------------------------------------------
publish('color', img)   --->  writes log (timestamp t1)   --->  puts to queue: (t1, 'image', img)
publish('depth', depth) --->  writes log (timestamp t2)   --->  puts to queue: (t2, 'depth', depth)
```

This leads to several critical issues:

1. **Timestamp Jitter & Desynchronization**:
   Because `serialize()` and log locking occur sequentially for each channel, `t1` and `t2` differ by milliseconds. Even if they arrive close in time, their recorded timestamps are not identical.
2. **Boilerplate & Fragile Matching in Consumers**:
   Any module requiring both streams (e.g., an RGB-D object detector or 3D point cloud mapper) cannot receive them together. Instead, it must:
   - Buffer incoming frames across separate `update()` iterations.
   - Guess or heuristically match frames using time windows (`abs(t_color - t_depth) < threshold`).
   - Suffer a 1-tick latency waiting for the partner stream.
   - Deal with edge cases like dropped frames, queue backpressure, or clock skew.
3. **Historical Workarounds in the Codebase**:
   - **Standalone Bundler Node (`_deprecated/subt/bundle_rgbd.py`)**: An intermediate node sat between camera and detector solely to collect `img`, `depth`, `camera_pose`, and `robot_pose`, dropping frames when desynchronized and outputting an `rgbd` tuple. This introduced thread overhead, extra queue hops, and configuration bloat.
   - **Sequence Sync Streams (`osgar/drivers/oak_camera_v3.py`)**: The driver registers side-channel sequence streams (`depth_seq`, `color_seq`, `detections_seq`) transmitting `[seq_num, timestamp_us]` before payload data to assist downstream synchronizers.
4. **The Monolithic Dilemma (The Anti-Pattern)**:
   If a driver packs data into a custom tuple/dict channel (e.g., `publish('rgbd', {'color': ..., 'depth': ...})`):
   - **Existing single-channel receivers break** (e.g., a simple video streamer expecting `color` cannot link directly to `camera.color`).
   - **Log inspection & tooling break** (`python -m osgar.logger --stream camera.color`, `LogReaderEx`, `tools/lidarview.py` cannot access individual streams).
   - **Compression is compromised**: Depth benefits from `:gz` (zlib compression) while Color is already compressed JPEG bytes. Bundling them into one monolithic msgpack payload forces all-or-nothing compression.
   - **Duplicating streams** (publishing both individual streams and a bundle stream) doubles log file size and memory bandwidth.

---

## 2. Core Architectural Requirements

| Requirement | Description |
| :--- | :--- |
| **R1. Strict Zero-Guessing Guarantee** | When published together, bundle elements share the exact same timestamp $t$ and are grouped deterministically without consumer heuristics. |
| **R2. Dual Consumption ("Slice vs. Whole")** | A consumer can subscribe to the **entire bundle** as an atomic unit, or to **individual components** without knowing a bundle was involved. |
| **R3. Log Storage Efficiency (Zero Duplication)** | Large payloads (e.g. RGB + Depth) MUST NOT be logged twice. Log files must store data once while preserving channel accessibility for standard log tools. |
| **R4. Channel-Specific Modifiers** | Modifiers like `:gz` and `:null` must continue to apply per individual stream (e.g., compressing depth while leaving JPEG color raw). |
| **R5. 100% Backward Compatibility** | Existing nodes calling `publish(channel, data)` and `on_<channel>(self, data)` require zero modifications. Configuration files with standard links continue to work. |
| **R6. Replay Determinism** | Replay via `osgar.replay` and `LogBusHandler` must reconstruct bundles with the exact same atomic delivery as the real-time runtime. |
| **R7. Multi-Runtime Parity** | Must function identically across standard threads (`_BusHandler`), ZMQ multi-process (`_Router` / `_Bus`), and replay (`LogBusHandler`). |

---

## 3. Design Space & Trade-Off Analysis

We evaluated three potential architectural paradigms:

```
Approach A: Composite Channel (Bundle-First)
  Source publishes 'rgbd' -> Bus slices into 'color' and 'depth' if requested.

Approach B: Receiver-Side Auto-Sync (Bus-Level Time Synchronizer)
  Source publishes separately -> Bus buffers and matches on arrival.

Approach C: First-Class Bundle Manifest with Atomic Multiplexing (Recommended)
  Source registers bundle association -> Atomic publish -> Multiplexed bus delivery.
```

### Approach Comparison Matrix

| Evaluation Criteria | Approach A: Composite Channel | Approach B: Bus-Level Auto-Sync | Approach C: First-Class Manifest (Recommended) |
| :--- | :--- | :--- | :--- |
| **Publishing Ergonomics** | `publish('rgbd', dict)` | Standard `publish()` calls | `publish_bundle('rgbd', dict)` or `publish_bundle(dict)` |
| **Single-Channel Consumption** | Requires virtual slicing in Bus | Native | Native (direct queue routing) |
| **Bundle Consumption** | Native | Requires buffering & matching in Bus | Native (atomic queue item) |
| **Log Storage & Duplication** | Monolithic stream (no dup, but breaks stream tools) | Separate streams (no dup, but timestamps differ) | **Zero duplication**: Component streams logged atomically with identical $t$; bundle metadata in Stream 0 |
| **Channel Compression (:gz)** | Difficult (applied to entire blob) | Native per channel | **Native per channel** (preserved) |
| **Replay Compatibility** | Easy for bundle, hard for legacy | Complex (requires fuzzy window matching in replay) | **Deterministic**: Shared $t$ and Stream 0 manifest eliminate matching ambiguity |
| **Tooling Impact (`osgar.logger`)** | ❌ Cannot view `camera.color` directly | ✔ Works | **✔ Works directly out of the box** |

**Conclusion:** **Approach C** is selected. It avoids data duplication, preserves per-channel compression modifiers, maintains complete compatibility with existing logging and replay tools, and guarantees zero-latency, zero-guessing atomic delivery to consumers.

---

## 4. Technical Architecture: First-Class Bundle with Atomic Multiplexing

### 4.1 System Diagram

```mermaid
graph TD
    subgraph "Producer Node (e.g. RealSense / OAK)"
        P[Driver Logic] -->|publish_bundle('rgbd', {color, depth})| BH[_BusHandler]
    end

    subgraph "OSGAR Bus (_BusHandler)"
        BH -->|Atomic dt generation| SYNC[Sync & Logger Lock]
        SYNC -->|Stream 1: color @ dt| LW[(LogWriter)]
        SYNC -->|Stream 2: depth:gz @ dt| LW
        SYNC -->|Direct delivery @ dt| Q1[Viewer Queue: 'image', color]
        SYNC -->|Direct delivery @ dt| Q2[SLAM Queue: 'depth', depth]
        SYNC -->|Bundled delivery @ dt| Q3[Detector Queue: 'rgbd', {color, depth}]
    end

    subgraph "Consumers"
        Q1 --> C1[Viewer Node<br/>on_image(color)]
        Q2 --> C2[SLAM Node<br/>on_depth(depth)]
        Q3 --> C3[RGB-D Detector Node<br/>on_rgbd(bundle)]
    end
```

---

### 4.2 Producer API Specification

#### Registration
A node registers its constituent output streams as usual, along with the bundle definition:

```python
class RealSense(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        # Register standard streams, including compression/null modifiers,
        # plus the bundle definition linking them:
        self.bus.register('color', 'depth:gz', 'infra',
                          bundles={'rgbd': ['color', 'depth']})
```
*Alternatively, a dedicated helper method can be provided:*
```python
self.bus.register('color', 'depth:gz', 'infra')
self.bus.register_bundle('rgbd', ['color', 'depth'])
```

#### Publishing
To emit bundled data atomically:

```python
# RealSense depth_callback:
self.publish_bundle('rgbd', {
    'color': color_jpeg_bytes,
    'depth': depth_numpy_array
})
```
*Convenience form:* If a node only defines a single bundle, the bundle name can be inferred:
```python
self.publish_bundle({'color': color_jpeg_bytes, 'depth': depth_numpy_array})
```

`publish_bundle` returns the assigned `timestamp` (`timedelta`).

---

### 4.3 Configuration & Linking Specification (`config.json`)

Links in `config.json` remain standard two-element string arrays `[sender.output, receiver.input]`:

```json
{
  "robot": {
    "modules": {
      "camera": {
        "driver": "osgar.drivers.realsense:RealSense",
        "init": { "device": "D400" }
      },
      "viewer": {
        "driver": "osgar.drivers.opencv:LogOpenCVCamera",
        "init": {}
      },
      "obstacle_detector": {
        "driver": "osgar.obstdet3d:ObstacleDetector3D",
        "init": {}
      },
      "ai_detector": {
        "driver": "subt.detector:Detector",
        "init": {}
      }
    },
    "links": [
      ["camera.color", "viewer.image"],
      ["camera.depth", "obstacle_detector.depth"],
      ["camera.rgbd", "ai_detector.rgbd"]
    ]
  }
}
```

* **Viewer** links to `camera.color` $\rightarrow$ receives only `color_jpeg_bytes`.
* **ObstacleDetector** links to `camera.depth` $\rightarrow$ receives only `depth_numpy_array`.
* **AIDetector** links to `camera.rgbd` $\rightarrow$ receives the dictionary `{'color': ..., 'depth': ...}`.

---

### 4.4 Consumer Node Handling

In the receiving node (`osgar.node.Node`), handler dispatch is unmodified:

```python
class Detector(Node):
    def on_rgbd(self, data):
        # 'data' is the dictionary published in the bundle:
        color = data['color']
        depth = data['depth']
        
        # Immediate processing with guaranteed temporal alignment:
        detections = self.model.predict(color, depth)
        self.publish('detections', detections)
```

For legacy nodes receiving individual channels, their handlers (`on_image`, `on_depth`) receive the payload directly as they do today.

---

### 4.5 Logfile Storage & Zero-Redundancy Mechanism

1. **Stream Registration in Stream 0 (INFO)**:
   In `LogWriter.register`, Stream 0 records stream names as it does today:
   ```python
   {'names': ['camera.color', 'camera.depth', 'camera.infra']}
   ```
   When a bundle is registered, `LogWriter` writes the manifest to Stream 0:
   ```python
   {'bundles': {'camera.rgbd': ['camera.color', 'camera.depth']}}
   ```
   *Note:* Existing `lookup_stream_names()` ignores any Stream 0 dictionary that does not have the `'names'` key. Thus, this addition causes zero parse errors in legacy readers.

2. **Packet Writing on `publish_bundle`**:
   The bus handler writes packets for each component channel in the bundle using a **single atomic lock and identical timestamp $t$**:
   ```python
   with self.logger.lock:
       dt = self.logger.get_time()  # Shared timestamp for the entire bundle
       for channel, data in bundle.items():
           stream_id = self.stream_id[channel]
           to_write = serialize(data)
           if stream_id in self.compressed_output:
               to_write = compress(to_write)
           self.logger.write(stream_id, to_write, dt=dt)
   ```
   * Result: **Zero redundant bytes stored on disk.**
   * `camera.color` is readable via `osgar.logger --stream camera.color`.
   * `camera.depth` is compressed with `:gz` and readable via `LogReaderEx`.
   * Both packets are stored contiguously in the log file with identical microsecond timestamps.

---

### 4.6 Replay Engine Architecture (`osgar.bus.LogBusHandler` & `osgar.replay`)

During log replay, a node may listen to a bundle channel (`camera.rgbd`), but the log contains only constituent streams (`camera.color` and `camera.depth`).

#### Replay Resolution Algorithm
1. **Manifest Extraction**:
   `osgar.replay` reads the bundle manifest from Stream 0 or resolves it from the registered driver configuration.
2. **Input Mapping**:
   If module `detector` links `camera.rgbd -> detector.rgbd`:
   `inputs` maps both `stream_id(camera.color)` and `stream_id(camera.depth)` to bundle group `rgbd`.
3. **LogBusHandler Buffer Aggregation**:
   When `LogBusHandler.listen()` reads from the log:
   - When Stream 1 (`camera.color`) arrives with timestamp `dt`: it is placed in an active bundle accumulator for `(rgbd, dt)`.
   - When Stream 2 (`camera.depth`) arrives with the same `dt`: the accumulator is complete.
   - `listen()` returns `(dt, 'rgbd', {'color': color, 'depth': depth})` to the caller.
4. **Dropped Frame & Incomplete Bundle Safety**:
   If a packet with `dt_next > dt` arrives before all bundle constituents have been read, an incomplete bundle warning is logged and the partial frame is flushed or dropped according to configuration.

---

### 4.7 Multi-Process Architecture (`osgar/zmqrouter.py`)

For multi-process execution via ZMQ:
1. **Registration Protocol**:
   `_Bus.register` transmits bundle manifests during initial handshake:
   `sock.send_multipart([b"register", b"color", b"depth:gz", b"bundle:rgbd=color,depth"])`
2. **Publication Protocol**:
   `_Bus.publish_bundle` serializes components and sends a single multi-part frame:
   `[b"publish_bundle", b"rgbd", b"color", raw_color, b"depth", raw_depth]`
3. **Router Routing**:
   `_Router.publish_bundle`:
   - Computes single timestamp `dt`.
   - Writes individual streams to `LogWriter` with `dt`.
   - Delivers component messages to single-channel subscribers.
   - Delivers composite multipart frame to bundle subscribers.

---

## 5. Case Study: Camera with Pose (Fixed vs. Steered / Servo Cameras)

A primary use case for message bundling is associating an image with the spatial orientation or 6-DOF pose of the camera at the moment of exposure.

### 5.1 The Dilemma: Fixed vs. Steered Cameras
1. **Fixed Camera**: Rigidly mounted to the robot chassis (e.g. front camera, rear camera, multi-camera surround). The orientation and translation relative to the robot base are static.
2. **Steered Camera**: Mounted on a pan-tilt servo, gimbal, or rotating mast (like surveillance PTZ cameras). The orientation changes continuously as the camera scans or tracks targets.

### 5.2 Architectural Decision: Publishing Constant Pose with Every Frame
For fixed cameras, should the driver publish a constant orientation/pose with every frame? **Yes, absolutely.**

#### Technical Rationale:
1. **Uniform Consumer Interface (Zero Branching)**:
   Downstream nodes (e.g. 3D object detectors, semantic segmentation, point cloud reprojectors) consume the bundle uniformly:
   ```python
   def on_camera(self, data):
       img, camera_pose = data  # Uniform across ALL camera types
   ```
   Consumer modules do not need conditional logic or configuration flags to check whether the robot has a fixed camera (reading a static parameter) or a steered camera (listening to a servo stream).
2. **Negligible Storage & Bus Overhead (< 0.05%)**:
   - A 3D pose `[[x, y, z], [qx, qy, qz, qw]]` serializes to **~35 bytes** in msgpack.
   - A single JPEG frame is **50,000 to 200,000 bytes**; a depth frame is **500,000+ bytes**.
   - Repeatedly publishing the static pose adds under **0.05%** to log file size and takes negligible CPU time.
3. **Single Source of Truth for Calibration**:
   In legacy configurations, because raw cameras did not emit their pose, the camera mount pose had to be duplicated into every consumer's `init` parameters (e.g. duplicated across `detector`, `obstacle_detector`, etc.). If physical calibration shifted, multiple configuration entries had to be maintained. Emitting `pose` in the camera bundle centralizes calibration into the camera driver.

### 5.3 Implementation Patterns

#### Pattern A: Fixed Camera (Static Mount Pose)
The camera driver accepts `mount_pose` in its `init` parameters and emits it alongside each captured frame:
```json
"camera_front": {
  "driver": "osgar.drivers.opencv:OpenCVCamera",
  "init": {
    "device": 0,
    "mount_pose": [[0.25, 0.0, 0.15], [0, 0, 0, 1]]
  }
}
```
Driver implementation:
```python
class OpenCVCamera(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.mount_pose = config.get('mount_pose', [[0, 0, 0], [0, 0, 0, 1]])
        self.bus.register('image', 'pose', bundles={'camera': ['image', 'pose']})

    def process_frame(self, frame):
        self.publish_bundle('camera', {
            'image': frame,
            'pose': self.mount_pose
        })
```

#### Pattern B: Integrated Steered Camera (PTZ / Gimbal / Servo Feedback)
When the camera and pan-tilt mechanism are handled by the same hardware/SDK, the driver queries the current angle and computes the dynamic transform for each frame:
```python
def on_frame_received(self, frame):
    current_angle = self.read_servo_angle()
    current_pose = self.compute_camera_pose(current_angle)
    self.publish_bundle('camera', {
        'image': frame,
        'pose': current_pose
    })
```

#### Pattern C: Decoupled Steered Camera (Separate Camera & Servo Drivers)
When the camera is a standard USB camera (Driver A) and the servo is driven via CAN/Serial (Driver B):
* An intermediate **`CameraRig`** node subscribes to `camera.image` and `servo.angle`. Upon receiving each image, it pairs it with the latest interpolated servo angle and outputs the unified bundle `camera_rig.camera`.
* Both fixed and steered cameras present the exact same `['image', 'pose']` bundle interface downstream.

### 5.4 Coordinate Frame Conventions
It is critical to distinguish between coordinate frames:
1. **`camera_to_robot` (Camera Extrinsic Pose)**:
   * **Included in the camera bundle**: Represents the position and orientation of the optical frame relative to the robot's base body center.
   * Constant for fixed cameras; dynamic for steered cameras.
   * Enables consumers to project camera detections directly into the robot's body coordinates.
2. **`robot_to_world` (Global Odometry / SLAM Pose)**:
   * **NOT included in the camera bundle**: Originates from platform odometry/wheel encoders/IMU at its own update rate.
   * Consumers requiring global coordinates (e.g. artifact mapping) subscribe to both `camera.camera` (for `camera_to_robot`) and `platform.pose3d` (for `robot_to_world`).

---

## 6. Detailed Implementation Handover Specification

### Phase 1: Core Bus & Logger (`osgar/bus.py`, `osgar/logger.py`)

#### 1.1 `osgar/bus.py`: `_BusHandler`
* Add bundle registry tracking in `_BusHandler.__init__`:
  ```python
  self.bundles = {}        # bundle_name -> [channel_1, channel_2, ...]
  self.bundle_subscribers = {} # bundle_name -> [(queue, input_channel), ...]
  ```
* Update `register()`:
  - Support keyword argument `bundles=None` or method `register_bundle(name, channels)`.
  - Validate that all channels listed in the bundle exist in `self.stream_id`.
  - Record the bundle in `self.logger` (if logger supports bundle registration).
* Update `connect()`:
  - If `output in self.bundles`: store subscriber in `self.bundle_subscribers[output]`.
* Implement `publish_bundle(bundle_name, data_dict)`:
  - Acquire `with self.logger.lock:`.
  - Calculate single `timestamp`.
  - For each `channel, payload` in `data_dict`:
    - Serialize, compress (if configured), and write to log via `self.logger.write(stream_id, data, dt=timestamp)`.
    - Dispatch to individual subscribers in `self.out[channel]`.
    - Dispatch to individual slots in `self.slots[channel]`.
  - For `queue, input_channel` in `self.bundle_subscribers.get(bundle_name, [])`:
    - `queue.put((timestamp, input_channel, data_dict))`
  - For bundle slots: invoke with `(timestamp, data_dict)`.
  - Return `timestamp`.

#### 1.2 `osgar/logger.py`: Bundle Manifest in Stream 0
* Add `register_bundle(bundle_name, stream_names)` to `LogWriter`:
  ```python
  def register_bundle(self, bundle_name, stream_names, dt=None):
      with self.lock:
          self.bundles[bundle_name] = stream_names
          self.write(stream_id=INFO_STREAM_ID,
                     data=bytes(str({'bundles': self.bundles}), encoding='ascii'),
                     dt=dt)
  ```
* Ensure `lookup_stream_names()` and other tools safely ignore non-`names` dicts in Stream 0.
* Add helper `lookup_stream_bundles(filename)` to extract bundle metadata.

---

### Phase 2: Node Layer (`osgar/node.py`)

* Add delegation method to `Node`:
  ```python
  def publish_bundle(self, bundle_name, data=None):
      if data is None and isinstance(bundle_name, dict):
          # Convenience overload: publish_bundle({'color': ..., 'depth': ...})
          data = bundle_name
          bundle_name = next(iter(self.bus.bundles.keys()))
      return self.bus.publish_bundle(bundle_name, data)
  ```

---

### Phase 3: Replay Engine (`osgar/bus.py`, `osgar/replay.py`)

#### 3.1 `osgar/bus.py`: `LogBusHandler`
* Add bundle accumulator in `LogBusHandler`:
  ```python
  self.bundle_accumulators = {}  # (bundle_name, dt) -> {channel: data}
  self.bundle_definitions = {}   # bundle_name -> set(channel_names)
  ```
* In `listen()`:
  - Read next stream from log.
  - If stream belongs to an input bundle:
    - Add to accumulator for `(bundle_name, dt)`.
    - If all required channels for `bundle_name` at `dt` have arrived:
      - Return `(dt, input_channel, completed_bundle_dict)`.
  - If stream is a normal input: return `(dt, channel, data)` as before.

#### 3.2 `osgar/replay.py`
* In `replay()`:
  - Detect links where `edge_from` corresponds to a bundle.
  - Register all constituent streams of the bundle in `inputs`.
  - Pass bundle definitions to `LogBusHandler`.

---

### Phase 4: Multi-Process ZMQ Router (`osgar/zmqrouter.py`)

* Update `_Bus`:
  - Add `publish_bundle(bundle_name, data_dict)`.
  - Support bundle registration in `register()`.
* Update `_Router`:
  - Support `b"publish_bundle"` command in `_Router.receive()` and dispatch.

---

### Phase 5: Driver Migration & Verification

* **Update Target Sensor Drivers**:
  - `osgar/drivers/realsense.py`: Migrate `depth_callback` to emit `rgbd` bundle.
  - `osgar/drivers/oak_camera_v3.py`: Migrate synchronized queues (color, depth, detections) to `publish_bundle()`, eliminating the need for `*_seq` workaround streams.
  - `osgar/drivers/lord_imu.py`: Bundle rotation and orientation if emitted simultaneously.
* **Deprecate Intermediate Nodes**:
  - Deprecate `_deprecated/subt/bundle_rgbd.py` in favor of native bus bundling.

---

## 7. Verification & Test Plan

1. **Unit Tests (`osgar/test_bus.py`)**:
   - `test_publish_bundle_individual_subscribers`: Verify that subscribing to `camera.color` receives individual payloads when `publish_bundle` is called.
   - `test_publish_bundle_bundle_subscriber`: Verify that subscribing to `camera.rgbd` receives the combined dictionary.
   - `test_publish_bundle_shared_timestamp`: Verify that individual and bundle subscribers receive the exact same microsecond timestamp.
   - `test_publish_bundle_compression`: Verify that `:gz` compression works on individual streams within a bundle.
2. **Replay Tests (`osgar/test_replay.py`)**:
   - Record a test log containing a bundle publish.
   - Replay a mock node subscribed to the bundle and verify that `on_bundle` receives the assembled dictionary.
   - Replay a mock node subscribed only to a constituent stream and verify it receives only that stream.
3. **Integration Test with Mock Drivers**:
   - Create a test pipeline with 1 camera emitting RGB-D bundles, 1 legacy viewer, 1 legacy depth processor, and 1 RGB-D detector running concurrently. Verify zero dropped frames and matching timestamps across all modules.

---

## 8. Open Points & Discussion Topics

### 8.1 Producer-Consumer Decoupling vs. Dictionary Key Naming
In traditional single-channel OSGAR links:
```json
["camera.color", "app.image"]
```
The link handles renaming: the producer calls its output `color`, but the consumer expects `image` and defines `on_image(data)`. The consumer is completely agnostic to how the producer named the stream.

With bundles, if the payload is passed as a raw dictionary `data = {'color': img, 'depth': d}`, a consumer accessing `data['color']` becomes tightly coupled to the producer's chosen dictionary keys. If another driver names its output `{'image': img, 'depth': d}` or `{'rgb': img, 'depth': d}`, the consumer breaks with a `KeyError`.

#### Candidate Solutions for Community / User Discussion:

1. **Option A: Positional Unpacking via `Bundle` Container (Pythonic & Decoupled)**
   The bus delivers a lightweight `Bundle` object that inherits from `tuple` (ordered according to the bundle's registration order) while still allowing dictionary/attribute access:
   ```python
   # Producer registers order: ['color', 'depth']
   self.bus.register_bundle('rgbd', ['color', 'depth'])
   ```
   The consumer can unpack positionally without caring about key names:
   ```python
   def on_rgbd(self, data):
       img, depth = data  # 100% decoupled from key names
   ```
   *Pros:* Extremely clean, backward compatible with tuple-style APIs, requires zero configuration changes.
   *Cons:* Depends on consistent element ordering across different drivers.

2. **Option B: Link-Level Key Remapping in `config.json`**
   The configuration link syntax is extended to optionally define key translations:
   ```json
   ["camera.rgbd", "app.rgbd", {"color": "image", "depth": "depth_map"}]
   ```
   When delivering to `app`, the bus remaps the dictionary keys so `app` receives `{'image': ..., 'depth_map': ...}`.
   *Pros:* Explicit, declarative, keeps consumer logic strictly isolated from driver variations.
   *Cons:* Modifies link syntax from 2-element tuples to optional 3-element tuples.

3. **Option C: Sub-Channel Link Mapping**
   Instead of linking the bundle name directly, links connect individual producer outputs into sub-keys of the consumer's input channel:
   ```json
   "links": [
     ["camera.color", "app.rgbd.image"],
     ["camera.depth", "app.rgbd.depth"]
   ]
   ```
   The bus handler groups incoming bundled channels into `app.rgbd` with the mapped keys `{'image': ..., 'depth': ...}`.
   *Pros:* Reuses the familiar channel naming pattern; supports combining outputs into customized target bundles.
   *Cons:* More verbose configuration files; bus handler must manage multi-link grouping.

4. **Option D: Standardized Core Schemas (Conventions)**
   Establish strict OSGAR naming conventions for canonical bundles:
   * `rgbd`: keys must always be `{'color', 'depth'}`
   * `imu`: keys must always be `{'acc', 'gyro'}`
   * `cam_pose`: keys must always be `{'image', 'pose'}`
   *Pros:* Zero runtime overhead, predictable across drivers.
   *Cons:* Less flexible for custom multi-sensor combinations.

---

## 9. Recommended Next Steps

With this analysis and architecture agreed upon, the implementation can proceed cleanly in incremental pull requests:
1. **PR 1: Core Bus & Logger**: Implement `register_bundle`, `publish_bundle`, and unit tests in `bus.py` and `logger.py`.
2. **PR 2: Replay Support**: Extend `LogBusHandler` and `replay.py` for bundle reconstruction.
3. **PR 3: Driver Migration**: Update `RealSense` and/or `OakCamera` to use `publish_bundle`.
