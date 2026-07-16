# RYUGU ROV (Remotely Operated Vehicle) Control System

A ROS 2 Humble workspace for the RYUGU ROV (Remotely Operated Vehicle) control system, designed to control the underwater vehicle via MAVROS and ArduSub v4.5.7. This project is prepared for the **Kontes Kapal Indonesia (KKI) 2026** competition.

---

## 📌 Project Status (Work In Progress)
> [!NOTE]
> This workspace is under active development.
> 
> * **Completed/Ready:** Core communication bridge (GCS ↔ Jetson ↔ Pixhawk), dual webcam HTTP MJPEG streamer, telemetry reader, arming control, and manual thruster/gripper control.
> * **In-Progress/Unfinished:** QR Code scanning program and the autonomous mission program.

---

## 📂 Workspace Structure

```text
RYUGU-ROV/
├── deploy/
│   ├── install_mavlink_router.sh           # One-shot mavlink-router build & deploy script
│   └── mavlink-router/
│       ├── main.conf                       # mavlink-router config (→ /etc/mavlink-router/)
│       └── mavlink-router.service          # systemd unit (→ /etc/systemd/system/)
├── scripts/
│   └── verify_mavlink_router.sh            # End-to-end telemetry chain verification
├── src/
│   └── ryugu_control/                      # Main ROS 2 package (Python)
│       ├── config/
│       │   └── ardusub_params.yaml          # MAVROS/ArduSub configuration parameters
│       ├── launch/
│       │   ├── mavros_sub.launch.py        # Launch file for MAVROS (standalone)
│       │   ├── ryugu_production.launch.py  # Production launch (direct serial mode)
│       │   └── ryugu_GoProduction_QGC.launch.py  # Production launch with QGC (UDP mode)
│       ├── ryugu_control/                  # Node source codes
│       │   ├── __init__.py
│       │   ├── gcs_bridge_node.py          # GCS ↔ Jetson ↔ Pixhawk UDP communication bridge node
│       │   ├── webcam_streamer.py          # Dual USB webcam HTTP MJPEG streaming node
│       │   ├── test_arming_mode.py         # Node for arming & flight mode testing
│       │   ├── test_sensor_reader.py       # Node for telemetry sensor reading tests
│       │   └── test_thrusters_gripper.py   # Node for thruster & gripper servo movement tests
│       ├── package.xml                     # ROS 2 package dependencies and metadata
│       ├── setup.cfg
│       └── setup.py                        # Configuration of entry points for console_scripts
└── tests/                                  # External test & simulation scripts (Non-ROS)
    ├── comm/
    │   ├── gcs_simulator.py                # GCS UDP Transmitter Simulator
    │   └── test_gcs_comm.py                # Interactive CLI tool for UDP & CRC-16 testing
    └── streaming/
        └── webcam_streamer.py              # Standalone webcam streamer test script
```

---

## ⚙️ System Specifications & Communication Architecture

The communication link relies on a custom UDP protocol validated using **CRC-16/CCITT-FALSE** (polynomial: `0x1021`, init: `0xFFFF`).

```text
                         ┌──────────────────────────────────────────────┐
                         │  GCS Laptop (192.168.1.100)                  │
                         │  ┌──────────────────┐  ┌──────────────────┐  │
                         │  │ QGroundControl   │  │ Custom GCS App   │  │
                         │  │ (IMU/Compass Cal)│  │ (UDP CMD & Telem)│  │
                         │  └────────┬─────────┘  └────────┬─────────┘  │
                         │           │                     │            │
                         └───────────┼─────────────────────┼────────────┘
                                     │ UDP :14550          │ UDP :5001 / :5002
                                     │ (MAVLink)           │ (Custom Binary + CRC16)
                                     │                     │
                         ┌───────────┼─────────────────────┼─────────────┐
                         │       Jetson Orin Nano (192.168.1.10)         │
                         │           │                     │             │
                         │  ┌────────▼─────────┐  ┌────────▼─────────┐   │
                         │  │ mavlink-router   │  │ gcs_bridge_node  │   │
                         │  │ (systemd service)│  │ (ROS2 Node)      │   │
                         │  └───┬─────────┬────┘  └───────┬──────────┘   │
                         │      │         │               │              │
                         │      │         │ UDP :14555    │              │
                         │      │         │ (MAVLink)     │              │
                         │      │  ┌──────▼───────┐       │              │
                         │      │  │ MAVROS Node  │◄──────┘              │
                         │      │  │(fcu_url: UDP)│ ROS2 topics/services │
                         │      │  └──────────────┘                      │
                         │      │                                        │
                         │  USB Serial                                   │
                         │  /dev/ttyACM0                                 │
                         │      │                                        │
                         │  ┌───▼───────────┐                            │
                         │  │ Dual Webcams  │  MJPEG HTTP                │
                         │  │ :8554/:8555   │                            │
                         │  └───────────────┘                            │
                         └───────────────────────────────────────────────┘
                                      │
                                 USB Serial
                              /dev/ttyACM0 @ 115200
                                      │
                         ┌────────────▼──────────────┐
                         │ Pixhawk 2.4.8             │
                         │ (ArduSub v4.5.7)          │
                         └───────────────────────────┘
```

### Uplink (GCS ➔ Pixhawk)
* `CMD_MOTION` (`0x81`): Manual motion control (pitch, roll, throttle, yaw, lateral, forward) @ 10 Hz
* `CMD_MODE` (`0x82`): Changes flight mode (STABILIZE, ALT_HOLD, MANUAL, etc.)
* `CMD_GRIPPER` (`0x83`): Controls gripper servo via MAV_CMD_DO_SET_SERVO
* `CMD_ARM` (`0x85`): Enables/disables motors (ARM/DISARM)
* `CMD_ESTOP` (`0x86`): Emergency Stop (instantly disarm)

### Downlink (Pixhawk ➔ GCS)
* `TELEM_IMU` (`0x01`): Pitch, roll, yaw orientation telemetry (°) @ 20 Hz
* `TELEM_DEPTH` (`0x02`): Depth and altitude telemetry @ 20 Hz
* `TELEM_STATUS` (`0x03`): Battery voltage, arming status, flight mode, and thruster RPM telemetry @ 20 Hz

---

## 🛠️ Installation & Prerequisites

### 1. System Dependencies
Ensure your environment is running **Ubuntu 22.04 LTS** with **ROS 2 Humble Hawksbill**. 
Install the following dependencies:
```bash
sudo apt update
sudo apt install python3-opencv python3-numpy ros-humble-mavros ros-humble-mavros-msgs -y
```

### 2. Build the Workspace
Clone this repository, then run `colcon build`:
```bash
# Go to workspace root
cd ~/RYUGU-ROV

# Build the package
colcon build --symlink-install

# Source the setup bash
source install/setup.bash
```

---

## 🚀 Running the Nodes

### 1. Production Launch (All Nodes Active)
Launches MAVROS, the GCS communication bridge, and the webcam streamer all together:
```bash
source install/setup.bash
ros2 launch ryugu_control ryugu_production.launch.py
```

### 2. Running Individual Nodes

* **GCS Communication Bridge Node:**
  ```bash
  ros2 run ryugu_control gcs_bridge_node
  ```
  *(Optional custom Jetson IP)*:
  ```bash
  ros2 run ryugu_control gcs_bridge_node --ros-args -p jetson_ip:=192.168.1.10
  ```

* **Dual Webcam MJPEG Streamer Node:**
  ```bash
  ros2 run ryugu_control webcam_streamer
  ```
  Video streams can be accessed via:
  * Front Camera: `http://<JETSON_IP>:8554/video`
  * Bottom Camera: `http://<JETSON_IP>:8555/video`
  * *Note: If a camera is offline, the stream will automatically serve a dark placeholder image labeled "CAMERA DISCONNECTED".*

---

## 🔀 MAVLink Routing & QGroundControl Calibration

The `mavlink-router` daemon splits the Pixhawk MAVLink telemetry stream in parallel to **MAVROS** (for the ROS2 control stack) and **QGroundControl** (for real-time sensor calibration on the GCS laptop).

### Architecture

```
Pixhawk /dev/ttyACM0 @ 115200
        │
        ▼
  mavlink-router (systemd service)
        │
        ├──► UDP 127.0.0.1:14555  ──► MAVROS  ──► ROS2 nodes
        │
        └──► UDP 192.168.1.100:14550  ──► QGroundControl (GCS laptop)
```

### 1. Install mavlink-router on Jetson

```bash
# One-shot build & deploy (compiles from source via meson + ninja)
cd ~/RYUGU-ROV
sudo ./deploy/install_mavlink_router.sh
```

The script installs `mavlink-routerd` to `/usr/local/bin`, deploys the config to `/etc/mavlink-router/main.conf`, and enables the systemd service.

### 2. Manage the mavlink-router Service

```bash
sudo systemctl start mavlink-router     # Start the router
sudo systemctl status mavlink-router    # Check if it's running
sudo systemctl stop mavlink-router      # Stop the router
sudo systemctl restart mavlink-router   # Restart after config changes
journalctl -u mavlink-router -f         # Follow live logs
```

### 3. Launch the ROS2 Stack (QGC-Compatible Variant)

This launch file sets MAVROS to listen on UDP port 14555 (where mavlink-router delivers the Pixhawk stream) instead of reading the serial port directly:

```bash
source install/setup.bash
ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py
```

**Fallback to direct serial** (if mavlink-router is not running):
```bash
ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py fcu_url:=/dev/ttyACM0:115200
```

The original `ryugu_production.launch.py` is preserved for direct-serial-only operation without QGC.

### 4. Connect QGroundControl on the GCS Laptop

1. Open **QGroundControl** (daily build recommended for ArduSub).
2. Go to **Application Settings → Comm Links**.
3. Add a UDP link: port `14550`, server mode (QGC listens).
4. Once connected, the QGC top bar shows "ArduSub v4.5.7" with vehicle status.
5. Navigate to **Sensors → Calibrate Sensors** to calibrate IMU, compass, and level horizon.

### 5. Verify Everything Is Working

```bash
# Run the verification suite on the Jetson:
./scripts/verify_mavlink_router.sh
```

This script checks the mavlink-router binary, systemd service, UDP ports, serial device, and MAVROS IMU telemetry — with clear PASS/FAIL indicators and suggested fixes.

---

## 🧪 Testing & Simulation Scripts

To simplify debugging without full hardware deployment, you can use the test scripts inside the `tests/` directory:

1. **GCS Communication Test (GCS Laptop Simulator):**
   On the GCS Laptop side, run this interactive CLI tool to send manual commands:
   ```bash
   python3 tests/comm/test_gcs_comm.py
   ```
2. **Arming Test Node:**
   ```bash
   ros2 run ryugu_control test_arming_mode
   ```
3. **Thrusters & Gripper Test Node:**
   ```bash
   ros2 run ryugu_control test_thrusters_gripper
   ```
4. **Sensor Telemetry Test Node:**
   ```bash
   ros2 run ryugu_control test_sensor_reader
   ```
