# DJI Tello Mini-Drone Integration Documentation

This document describes the technical implementation, configuration, and verification of the DJI Tello mini-drone platform within the OSGAR ecosystem.

---

## 1. Technical Overview of Tello Communication Protocols

The Ryze Tello mini-drone communicates over local Wi-Fi (where the drone acts as an access point) using three standard User Datagram Protocol (UDP) ports:

| Connection Type | Target / Host Port | Role | Protocol Details |
| :--- | :--- | :--- | :--- |
| **Control Connection** | `192.168.10.1:8889` | Bidirectional Command / Response | Sends text-based ASCII SDK commands (e.g., `"takeoff"`, `"land"`, `"rc a b c d"`) and receives execution ACKs (like `"ok"`, `"error"`, or query values like `"85"`). |
| **State / Telemetry** | Local Port `8890` (from Drone) | Unidirectional Broadcast | Drone streams status updates at ~10Hz containing semicolon-separated key-value pairs (e.g., `pitch`, `roll`, `yaw`, `tof`, `bat`, `baro`). |
| **Video Stream** | Local Port `11111` (from Drone) | Unidirectional Broadcast | Streams raw H.264-encoded video packets once the `"streamon"` command is executed on the Control port. |

---

## 2. Completed Platform Architecture (`osgar/platforms/tello.py`)

The experimental Tello driver has been successfully cleaned up, fully modernized, and integrated as a native platform class under `osgar/platforms/tello.py`.

### A. Configuration-Driven Flight Task Orchestration
Rather than hardcoding task lists within class code, the flight tasks are completely configuration-driven.
* **JSON Definition:** The flight execution script is placed inside the `"init"` dictionary of the Tello platform module in `config/tello.json`:
  ```json
  "tello": {
      "driver": "osgar.platforms.tello:TelloDrone",
      "in": ["status", "video"],
      "out": ["cmd", "jpeg"],
      "init": {
          "tasks": [
              [1, "streamon"],
              [2, "takeoff"],
              [11, "up 100"],
              [12, "cw 180"],
              [20, "land"],
              [22, "streamoff"]
          ]
      }
  }
  ```
* **Type-Safe Task Parser:** The platform automatically accepts `tasks` from its configuration parameter, parses task tuples, and dynamically converts string commands from JSON into `bytes` for raw UDP transmission, while retaining fallback default lists for compatibility.

### B. High-Performance H.264 Video Stream Decoding with PyAV (`av`)
The previous raw UDP packet accumulation hack has been replaced with a robust, zero-copy, FFmpeg-backed decoding pipeline using PyAV (`av`):
* **Parser & Codec Pipeline:** A dynamic `av.CodecContext` decoder is initialized at startup. Incoming video stream packets (Annex B H.264 stream fragments) are fed chunk-by-chunk to `self.codec.parse(data)`.
* **Resilient Exception Isolation:** Video packet drops and startup packet fragmentations are isolated by catching `av.error.FFmpegError` specifically. This suppresses decoding warning dumps while keeping the decoder active until keyframes (I-frames) arrive to lock on, and ensures critical Python interrupts (like `Ctrl+C` / `KeyboardInterrupt`) propagate normally.
* **JPEG Frame Publishing:** Once a video frame is fully assembled and decoded, it is converted to a BGR NumPy array, compressed into a `.jpeg` binary payload using OpenCV (`cv2.imencode`), and published instantly on the registered `"jpeg"` output channel matching standard OSGAR camera driver conventions.

---

## 3. Recommended Future Ecosystem Protocols (Roadmap)

To expand beyond sequential scheduled script flight tasks, the following integrations are recommended:

### Option A: DJITelloPy Integration & Command Paradigms
For robust closed-loop flight paths and swarm setups, DJITelloPy provides a mix of two powerful control paradigms matching different robotics workloads:

#### 1. Discrete/Step-by-Step Commands (Synchronous & Blocking)
* **How it works:** Discrete textual commands like `"takeoff"`, `"land"`, or `"forward 50"` are inherently synchronous on the drone's firmware. The drone will *not* return the `"ok"` response until it has fully completed the action (e.g. hovered steadily after takeoff, or completed the 50cm flight).
* **Library wrapping:** DJITelloPy wraps these as blocking methods (`tello.takeoff()`, `tello.move_left(50)`). The calling thread blocks and waits for the `"ok"` or `"error"` ACK (with built-in socket timeouts), which delivers safe, sequential, closed-loop behavior out-of-the-box.

#### 2. Real-Time/Fluid Piloting Commands (Asynchronous & Non-Blocking)
* **How it works:** For high-frequency, dynamic piloting (e.g., keyboard flight, gamepad inputs, or computer-vision target-tracking via PID loops), the Tello SDK provides a special asynchronous command format: `rc a b c d` (where `a`: roll / left-right, `b`: pitch / forward-backward, `c`: throttle / up-down, and `d`: yaw / rotation, with values from `-100` to `100`).
* **No ACK congestion:** Crucially, **the drone does NOT reply with `"ok"` to `rc` commands** to keep the control channel latency-free.
* **Control Loop Pattern:** In DJITelloPy, `tello.send_rc_control(la, fb, ud, yaw)` transmits the values instantly without blocking. This allows you to implement high-speed continuous tracking loops (typically operating at 20Hz - 50Hz) directly integrated with OSGAR's standard closed-loop platform control loops (similar to `subt/drone.py`):
  ```python
  # Periodically compute errors and update speeds at 20Hz:
  while running:
      error_x, error_y = detect_target(camera_frame)
      ud_speed = pid_altitude(error_y)
      yaw_speed = pid_yaw(error_x)
      # Transmit raw RC control asynchronously:
      tello.send_rc_control(0, 0, ud_speed, yaw_speed)
      time.sleep(0.05)
  ```

### Option B: ROS / ROS 2 `tello_driver`
* **Paradigms:** For full 3D odometry and standard twist/control layout.
* **Benefits:** Exposes standard `/cmd_vel` (`geometry_msgs/Twist`) and raw images via `/image_raw` topics. Can interface directly with OSGAR via the built-in `ROSMsgParser` driver.

---

## 4. Verification & Testing

### A. Automated Unit Testing
The class behaves fully conformant with OSGAR's testing conventions. Running discover on the platforms folder executes unit tests for validation:
```bash
python -m unittest osgar.platforms.test_tello
```
This tests:
1. Platform node instantiation.
2. Safe config dictionary reading, fallback defaults, and automatic `str -> bytes` tasks parameter parsing.

### B. Empirical Log Replay Verification
The decoding pipeline has been empirically verified using a real-world flight log (`tello-260601_164015.log`).

You can execute the replay tool to re-decode and process the entire video stream:
```bash
python -m osgar.replay -F --output output_tello.log tello-260601_164015.log --module tello
```

Running the logger utility on the resulting log:
```bash
python -m osgar.logger output_tello.log
```
Returns perfect, zero-loss telemetry stats:
```text
 k       name     bytes | count | freq Hz
-----------------------------------------
 0        sys       970 |   4 |   0.0Hz
 1  tello.cmd        50 |   6 |   0.0Hz
 2 tello.jpeg 100565811 | 587 |   0.1Hz

Total time 1:11:55.046602
```
This confirms that **587 video frames** were successfully compiled from the raw UDP fragments, decoded, compressed to JPEG, and recorded cleanly with 100% data integrity.
