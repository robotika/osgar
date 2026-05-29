# Simulator Module Analysis and Plan

## Objective
Extend the OSGAR framework with a `simulator` functionality. The primary goal is to feed sensor data from recorded logs (re-simulation) while accepting new control commands, effectively allowing closed-loop testing of control algorithms against recorded datasets.

## Background & Motivation
Currently, OSGAR supports single-module replay (`osgar/replay.py`), which replays logged inputs to a specific node but does not support full-system closed-loop re-simulation where control changes impact the simulated state. Introducing a dedicated Simulator node will bridge this gap, allowing developers to test new control logic on historical data and observe how the system would have responded dynamically.

## Key Architecture Concepts
1. **Live Execution & Bus**: In record/live mode, nodes communicate over a concurrent `Bus`.
2. **Replay**: `LogReader` reads log files sequentially by stream ID.
3. **LogSimulator Node**: A new custom driver node configured in standard OSGAR JSON.
   - **Inputs**: Control command streams (e.g., `desired_speed`, `steering`, `move`).
   - **Outputs**: Simulated/replayed sensors (e.g., `scan`, `camera`, `imu`, `pose2d`).
   - **State**: Maintains a virtual kinematics model updated by the input controls.

## Selected Approach: Standalone Simulator Node (`LogSimulator`)
Create a new node in `osgar/simulator/` (or `osgar/drivers/log_simulator.py`).
- **Functionality**: Takes a `.log` file path as an initialization parameter.
- **Playback**: Sequentially reads the log file and publishes replayed sensor streams (LIDAR, camera, IMU).
- **Kinematics & Control**: Receives control commands on its inputs, updates a virtual kinematics model, and publishes dynamic odometry (`pose2d`/`pose3d`) instead of replaying recorded odometry.
- **Timing**: Replays sensor data synchronized with the delta time (`dt`) between consecutive log timestamps, allowing the driving nodes to operate in real-time.

---

## Sensor Re-simulation & View Synthesis Strategies
When the simulated robot's trajectory diverges from the recorded log, raw replay is insufficient. We must creatively transform recorded data to match the new simulated pose.

### General Spatial Sensors (LIDAR, Depth Cameras)
1. **2D Scan Projection & Rigid Transformation (Recommended Starting Point)**
   - **Concept:** Convert recorded depth/LIDAR into a 2D array of distances (utilizing `osgar.lib.depth`).
   - **Simulation:** Calculate the *pose delta* (difference between simulated pose and recorded pose). Apply this rigid 2D translation/rotation to the scanned points, then re-project into a simulated 1D scan array.
2. **3D Point Cloud Reprojection (Advanced)**
   - **Concept:** Project recorded depth images into local 3D point clouds.
   - **Simulation:** Translate and rotate the entire 3D point cloud by the 3D pose delta, then project the shifted points back onto a 2D image plane to synthesize a new depth image.

### Edge-Compute Neural Sensors (e.g., Luxonis OAK-D Pro)
Devices like the OAK-D Pro (used in OSGAR configurations like Matty) run neural networks directly on the camera hardware, publishing `detections` (bounding boxes) directly to the bus, bypassing host-side processing.

1. **Recorded Bounding Box Re-projection (Lightweight / Recommended)**
   - **Concept:** The `LogSimulator` intercepts the logged `detections` stream.
   - **Simulation:** Treat detections as 3D objects (using their bounding box and depth). Apply the simulated *pose delta* to these objects. For example, if the simulated robot turns 10 degrees left relative to the log, shift the bounding boxes 10 degrees right in the simulated camera frame.
   - **Pros:** Extremely fast, requires no GPU/AI processing during simulation.

2. **Host-Side Inference on Synthesized Images (For Model Tuning)**
   - **Concept:** If testing a *new* neural network model is required, the simulator synthesizes a new image/depth frame (using 3D reprojection) and runs a software equivalent (PyTorch/OpenVINO) on the host CPU/GPU to generate fresh detections.

3. **Hardware-in-the-Loop (HIL) Injection**
   - **Concept:** Transform the recorded frames based on the pose divergence, and use the Luxonis DepthAI library to inject these host-generated frames *back into* a physical OAK-D connected via USB to run the inference.

---

## Implementation Plan

### Phase 1: Create `LogSimulator` Node
1. Create `osgar/simulator/` module.
2. Implement `LogSimulator` inheriting from `Node`.
3. Implement initialization parameters to accept `log_filepath`.
4. Use `LogReader` to parse and queue sensor messages from the log.

### Phase 2: Implement Kinematics and Control Update
1. Add input channels for control streams (`desired_speed`, `move`).
2. Implement a basic kinematic model (e.g., differential drive) to calculate dynamic `pose2d` based on integrated controls.
3. Implement a timing loop respecting the log's timestamps.

### Phase 3: Sensor Transformation (Divergence Handling)
1. Implement the **2D Scan Projection** strategy. Calculate pose delta between log and simulation, and transform LIDAR/Depth scans before publishing.
2. Implement **Bounding Box Re-projection** for AI sensors (like OAK-D detections).

### Phase 4: Integration and Examples
1. Create a sample JSON configuration (e.g., `config/resimulation.json`) demonstrating how to swap physical drivers with `LogSimulator`.
2. Provide a run script for executing the re-simulation.

## Verification & Testing
- Unit test checking that `LogSimulator` outputs transformed scan data when controls are injected that diverge from the log.
- Run a historical log with modified control logic and verify `pose2d` diverges correctly while sensor data dynamically adapts.