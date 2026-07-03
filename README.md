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
├── src/
│   └── ryugu_control/                      # Main ROS 2 package (Python)
│       ├── config/
│       │   └── ardusub_params.yaml          # MAVROS/ArduSub configuration parameters
│       ├── launch/
│       │   ├── mavros_sub.launch.py        # Launch file for MAVROS
│       │   └── ryugu_production.launch.py  # Main production launch file for the entire system
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
    GCS Laptop (192.168.1.100)
               │
      UDP Ports :5001 / :5002
               ▼
     Jetson Orin Nano (192.168.1.10)  ◄─── (Dual Webcams MJPEG HTTP :8554 & :8555)
               │
          USB Serial
               ▼
    Pixhawk 2.4.8 (ArduSub v4.5.7)
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
