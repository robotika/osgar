# DJI Tello Mini-Drone Integration Analysis & Roadmap

This document summarizes the technical findings, structural issues, and future-proofing recommendations for integrating the Ryze/DJI Tello mini-drone platform into the OSGAR ecosystem.

---

## 1. Technical Overview of Tello Communication Protocols

The Ryze Tello mini-drone communicates over local Wi-Fi (where the drone acts as an access point) using three standard User Datagram Protocol (UDP) ports:

| Connection Type | Target / Host Port | Role | Protocol Details |
| :--- | :--- | :--- | :--- |
| **Control Connection** | `192.168.10.1:8889` | Bidirectional Command / Response | Sends text-based ASCII SDK commands (e.g., `"takeoff"`, `"land"`, `"rc a b c d"`) and receives execution ACKs (like `"ok"`, `"error"`, or query values like `"85"`). |
| **State / Telemetry** | Local Port `8890` (from Drone) | Unidirectional Broadcast | Drone streams status updates at ~10Hz containing semicolon-separated key-value pairs (e.g., `pitch`, `roll`, `yaw`, `tof`, `bat`, `baro`). |
| **Video Stream** | Local Port `11111` (from Drone) | Unidirectional Broadcast | Streams raw H.264-encoded video packets once the `"streamon"` command is executed on the Control port. |

---

## 2. Assessment of the Experimental `feature/tello` Branch

The current legacy implementation (`osgar/drivers/tello.py` and `config/tello.json`) was a highly experimental proof-of-concept with several architectural limitations:

### A. Brittle Command Orchestration
* **Current Design:** Uses a simple time-scheduled sequence inside `self.tasks` (e.g. `[2, b'takeoff']`, `[11, b'up 300']`). 
* **The Flaw:** This approach is open-loop and timestamp-driven. It does not wait for a confirmation `"ok"` or `"error"` ACK from the drone before initiating the next task. If a command (like `takeoff`) takes longer than expected or is dropped due to packet loss, subsequent commands will fail or be misapplied.

### B. Lossy Video Stream Assembly
* **Current Design:** `on_video` accumulates raw incoming UDP datagrams in a local buffer and attempts to parse them directly.
* **The Flaw:** UDP is a packet-level, lossy transport. Standard H.264 video decoding requires systematic application-layer packet assembly, frame synchronization, and keyframe tracking. Simply appending raw UDP packets leads to corrupted frames and decoding failures.
* **Alternative Tooling:** The utility `osgar/tools/play_udp_video.py` bypasses this by spinning up `cv2.VideoCapture('udp://@127.0.0.1:11111')`. This delegatory design allows OpenCV's compiled FFmpeg engine to properly handle the UDP protocol, packet buffering, and H.264 decoding in real-time.

---

## 3. Recommended Ecosystem Protocols & Alternatives

To achieve robust flight control and reliable video capture, the following communication interfaces should be considered:

### Option A: DJITelloPy (Highly Recommended)
An actively maintained high-level Python library wrapping Ryze's official text SDK.
* **Synchronous Handshaking:** Command execution functions block until the corresponding `"ok"` is returned (or a timeout is hit), providing closed-loop control.
* **Keep-Alive Thread:** Tello automatically auto-lands if it does not receive a control command for 15 seconds. `DJITelloPy` manages this automatically in a background keep-alive thread.
* **Decoded Video Stream:** Employs PyAV or OpenCV to spin up a background thread that serves fully-decoded video frames as standard NumPy arrays.
* **Telemetry Parsing:** Automatically handles background socket listener threads for Port 8890, parsing data straight into object properties.

### Option B: TelloPy (Reverse-Engineered Binary Protocol)
An older, unmaintained Python library using reverse-engineered binary packets.
* **Analysis:** While it provides low-latency, event-driven callbacks, it is unmaintained and experiences dependency conflicts (such as legacy PyAV versions) on modern Python 3.10+ environments. It should be avoided for future iterations.

### Option C: ROS / ROS 2 Bridge (`tello_driver`)
A hybrid architecture running the official ROS/ROS 2 `tello_driver` in a sidecar container.
* **Analysis:** This encapsulates control commands as `/cmd_vel` (`geometry_msgs/Twist`) and exposes decoded images via `/image_raw`. Since OSGAR already contains a robust `ROSMsgParser` driver, it can seamlessly bind to this standard robotics layout.

---

## 4. Architectural Roadmap for OSGAR Platforms

To properly integrate Tello under OSGAR's modern conventions, the drone integration should be structured as follows:

```
                  +-------------------------------------------------+
                  |         osgar/platforms/tello.py (Platform)     |
                  |  - Translates high-level 3D movements to RC cmds |
                  +-----------------------+-------------------------+
                                          | (bus.publish 'cmd')
                                          v
                  +-------------------------------------------------+
                  |         osgar/drivers/tello.py (Driver)         |
                  |  - Manages UDP Control, State, and Video Ports |
                  +-----------------------+-------------------------+
                     /                    |                    \
                    /                     |                     \
 (UDP Port 8889)   /    (UDP Port 8890)   |      (UDP Port 11111) \
                  v                       v                        v
        +-----------+           +-----------+            +-------------------+
        |  Control  |           | Telemetry |            |  H.264 IP Camera  |
        |  Socket   |           |  Socket   |            | (OpenCV decoding) |
        +-----------+           +-----------+            +-------------------+
```

### 1. Separate Platform and Driver
* **Driver Node (`osgar/drivers/tello.py`):** Focuses strictly on managing the three UDP socket loops (or wrapping `DJITelloPy`).
* **Platform Node (`osgar/platforms/tello.py`):** Inherits from OSGAR's core platform class to unify interfaces. It translates standard 3D velocity vectors (`linear` X/Y/Z, `angular` yaw) into continuous joystick commands (`rc a b c d`) for smoother, non-stepped flight paths.

### 2. Standardized Video Publishing
Instead of writing raw byte dumps, the Tello driver's video thread should decode H.264 streams on the fly using `cv2.VideoCapture` and re-publish them as serialized JPEG frames on the standard `raw` channel, aligning with `osgar/drivers/opencv.py`:
```python
ret, frame = cap.read()
if ret:
    retval, data = cv2.imencode('*.jpeg', frame)
    if len(data) > 0:
        self.bus.publish('raw', data.tobytes())
```
