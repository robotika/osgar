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
- **Pros**: Clean isolation; acts as a drop-in replacement for physical drivers in JSON configs.

## Implementation Plan

### Phase 1: Create `LogSimulator` Node
1. Create `osgar/simulator/` module if it doesn't exist, and add the main node logic. (The final location of this plan will also be `osgar/simulator/PLAN.md` after implementation begins).
2. Implement `LogSimulator` inheriting from `Node`.
3. Implement initialization parameters to accept `log_filepath`.
4. Use `LogReader` to parse and queue sensor messages from the log.

### Phase 2: Implement Kinematics and Control Update
1. Add input channels for control streams (`desired_speed`, `move`).
2. Implement a basic kinematic model (e.g., differential drive or Ackermann) to calculate dynamic `pose2d` based on integrated controls.
3. Implement a timing loop that respects the log's timestamps, publishing sensors at the correct frequency.

### Phase 3: Integration and Examples
1. Create a sample JSON configuration (e.g., `config/resimulation.json`) demonstrating how to swap physical drivers with `LogSimulator`.
2. Provide a run script or documentation on executing the re-simulation.

## Verification & Testing
- Create a unit test checking that `LogSimulator` reads a dummy log and outputs data when controls are injected.
- Run a historical log with the original control logic and verify the output `pose2d` matches the log exactly.
- Run a historical log with modified control logic and verify `pose2d` diverges correctly while sensor data continues to be published.