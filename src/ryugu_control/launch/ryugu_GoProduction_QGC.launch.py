"""
ryugu_GoProduction_QGC.launch.py — Production Stack with QGroundControl Support

Launches all nodes required for competition operation with mavlink-router
splitting the Pixhawk telemetry to both MAVROS and QGroundControl:

  1. MAVROS node       (listens on UDP 127.0.0.1:14555 ← mavlink-router)
  2. GCS Bridge Node   (Jetson ↔ GCS Laptop UDP telemetry & commands)
  3. Webcam Streamer   (dual USB camera MJPEG over HTTP)

This launch file is the QGC-compatible variant of ryugu_production.launch.py.
It assumes mavlink-router is already running (systemd service) and routing:
  Pixhawk /dev/ttyACM0 → UDP 127.0.0.1:14555  (MAVROS)
                       → UDP 192.168.1.100:14550 (QGC)

Hardware:
  - Pixhawk 2.4.8 on /dev/ttyACM0 @ 115200 baud (via mavlink-router)
  - 6 Thrusters (DSHOT) on AUX 1–6
  - Gripper Servo (MG996R) on MAIN 1
  - 2 USB Webcams on /dev/video0 and /dev/video2

Network:
  - Jetson Orin Nano: 192.168.1.10
  - GCS Laptop:       192.168.1.100

Prerequisites:
  # 1. Install and start mavlink-router (see deploy/install_mavlink_router.sh):
  sudo systemctl start mavlink-router

  # 2. Launch the ROS2 stack:
  ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py

Usage:
  # Default (UDP localhost via mavlink-router):
  ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py

  # Fallback: direct serial (no mavlink-router):
  ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py \\
      fcu_url:=/dev/ttyACM0:115200

  # Override network:
  ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py \\
      jetson_ip:=192.168.2.10 gcs_ip:=192.168.2.100

  # Disable cameras (bridge-only mode):
  ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py enable_webcam:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────
    mavros_share = get_package_share_directory('mavros')
    config_yaml = os.path.join(mavros_share, 'launch', 'apm_config.yaml')
    pluginlists_yaml = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')

    # ═══════════════════════════════════════════════════════════════════════
    #  Launch arguments
    # ═══════════════════════════════════════════════════════════════════════

    # ── MAVROS arguments ─────────────────────────────────────────────────
    #
    #  IMPORTANT:  The default fcu_url is udp://127.0.0.1:14555@ which
    #  tells MAVROS to BIND and LISTEN on localhost port 14555 (server
    #  mode).  mavlink-router connects to this port as a client and
    #  forwards the Pixhawk MAVLink stream.
    #
    #  The '@' suffix is critical — it means "listen, don't connect".
    #  Without it MAVROS would try to reach OUT to that address.
    #
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14555@',
        description=(
            'FCU connection URL.  '
            'Default reads from mavlink-router via UDP.  '
            'Fallback serial: /dev/ttyACM0:115200  '
            'Fallback UDP all: udp://:14540@'
        ),
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='MAVLink GCS URL (leave empty — we use custom UDP bridge).',
    )
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system', default_value='1',
        description='MAVLink target system ID.',
    )
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component', default_value='1',
        description='MAVLink target component ID.',
    )
    system_id_arg = DeclareLaunchArgument(
        'system_id', default_value='1',
        description='MAVLink system ID for this MAVROS instance.',
    )
    component_id_arg = DeclareLaunchArgument(
        'component_id', default_value='240',
        description='MAVLink component ID (240 = companion computer).',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='mavros',
        description='ROS2 namespace for the MAVROS node.',
    )

    # ── Bridge arguments ─────────────────────────────────────────────────
    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip', default_value='192.168.1.10',
        description='Jetson Orin Nano IP address.',
    )
    gcs_ip_arg = DeclareLaunchArgument(
        'gcs_ip', default_value='192.168.1.100',
        description='GCS Laptop IP address.',
    )
    cmd_port_arg = DeclareLaunchArgument(
        'cmd_port', default_value='5001',
        description='UDP port for incoming GCS commands.',
    )
    telem_port_arg = DeclareLaunchArgument(
        'telem_port', default_value='5002',
        description='UDP port for outgoing telemetry to GCS.',
    )

    # ── Webcam arguments ─────────────────────────────────────────────────
    enable_webcam_arg = DeclareLaunchArgument(
        'enable_webcam', default_value='true',
        description='Enable the dual webcam streamer node.',
    )
    front_dev_arg = DeclareLaunchArgument(
        'front_dev', default_value='/dev/video0',
        description='Front camera V4L2 device.',
    )
    bottom_dev_arg = DeclareLaunchArgument(
        'bottom_dev', default_value='/dev/video2',
        description='Bottom camera V4L2 device.',
    )
    front_port_arg = DeclareLaunchArgument(
        'front_port', default_value='8554',
        description='Front camera HTTP stream port.',
    )
    bottom_port_arg = DeclareLaunchArgument(
        'bottom_port', default_value='8555',
        description='Bottom camera HTTP stream port.',
    )
    cam_width_arg = DeclareLaunchArgument(
        'cam_width', default_value='640',
        description='Camera capture width.',
    )
    cam_height_arg = DeclareLaunchArgument(
        'cam_height', default_value='480',
        description='Camera capture height.',
    )
    cam_fps_arg = DeclareLaunchArgument(
        'cam_fps', default_value='30',
        description='Camera capture/stream target FPS.',
    )

    # ═══════════════════════════════════════════════════════════════════════
    #  MAVROS Node
    # ═══════════════════════════════════════════════════════════════════════
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[
            pluginlists_yaml,
            config_yaml,
            {
                # ── Connection ──
                'fcu_url':        LaunchConfiguration('fcu_url'),
                'gcs_url':        LaunchConfiguration('gcs_url'),
                'tgt_system':     LaunchConfiguration('tgt_system'),
                'tgt_component':  LaunchConfiguration('tgt_component'),
                'system_id':      LaunchConfiguration('system_id'),
                'component_id':   LaunchConfiguration('component_id'),
                'fcu_protocol':   'v2.0',

                # ── Plugin denylist (disable unused to save CPU) ──
                'plugin_denylist': [
                    'actuator_control',
                    'obstacle_distance',
                    'vision_speed_estimate',
                    'safety_area',
                    'debug_value',
                    'trajectory',
                    'adsb',
                    'play_tune',
                    'landing_target',
                    'wind_estimation',
                ],

                # ── Connection timers ──
                'conn/heartbeat_rate':  1.0,
                'conn/timeout':        10.0,
                'conn/timesync_rate':  10.0,

                # ── IMU ──
                'imu/frame_id':                 'base_link',
                'imu/linear_acceleration_stdev': 0.0003,
                'imu/angular_velocity_stdev':    0.0003,
                'imu/orientation_stdev':         1.0,

                # ── Local position ──
                'local_position/frame_id':       'map',
                'local_position/tf/send':         True,
                'local_position/tf/frame_id':    'map',
                'local_position/tf/child_frame_id': 'base_link',

                # ── Global position ──
                'global_position/frame_id':       'map',
                'global_position/child_frame_id': 'base_link',
                'global_position/tf/send':        False,

                # ── Setpoint ──
                'setpoint_attitude/reverse_thrust': True,

                # ── System ──
                'sys/min_voltage':  10.0,
                'sys/disable_diag': False,
            },
        ],
    )

    # ═══════════════════════════════════════════════════════════════════════
    #  GCS Bridge Node
    # ═══════════════════════════════════════════════════════════════════════
    gcs_bridge_node = Node(
        package='ryugu_control',
        executable='gcs_bridge_node',
        name='gcs_bridge_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'jetson_ip':           LaunchConfiguration('jetson_ip'),
            'gcs_ip':              LaunchConfiguration('gcs_ip'),
            'cmd_port':            LaunchConfiguration('cmd_port'),
            'telem_port':          LaunchConfiguration('telem_port'),
            'manual_control_rate': 10.0,
            'telemetry_rate':      20.0,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════════
    #  Webcam Streamer Node  (conditionally launched)
    # ═══════════════════════════════════════════════════════════════════════
    webcam_node = Node(
        package='ryugu_control',
        executable='webcam_streamer',
        name='webcam_streamer',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(LaunchConfiguration('enable_webcam')),
        parameters=[{
            'bind':         '0.0.0.0',
            'front_port':   LaunchConfiguration('front_port'),
            'bottom_port':  LaunchConfiguration('bottom_port'),
            'front_dev':    LaunchConfiguration('front_dev'),
            'bottom_dev':   LaunchConfiguration('bottom_dev'),
            'width':        LaunchConfiguration('cam_width'),
            'height':       LaunchConfiguration('cam_height'),
            'fps':          LaunchConfiguration('cam_fps'),
            'jpeg_quality':   70,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════════
    #  Banner
    # ═══════════════════════════════════════════════════════════════════════
    banner = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════════════════════╗\n',
        '║        RYUGU ROV — Go-Production Stack (+ QGC Support)           ║\n',
        '║                                                                  ║\n',
        '║  FCU (via mavlink-router):  ', LaunchConfiguration('fcu_url'), '\n',
        '║  QGC on GCS:                 192.168.1.100:14550\n',
        '║  Bridge: ', LaunchConfiguration('jetson_ip'),
                       ':', LaunchConfiguration('cmd_port'),
                       '  →  GCS ', LaunchConfiguration('gcs_ip'),
                       ':', LaunchConfiguration('telem_port'), '\n',
        '║  Cameras: ', LaunchConfiguration('enable_webcam'), '\n',
        '║                                                                  ║\n',
        '║  Stream URLs (when cameras enabled):                             ║\n',
        '║    Front:  http://', LaunchConfiguration('jetson_ip'),
                       ':', LaunchConfiguration('front_port'), '/video\n',
        '║    Bottom: http://', LaunchConfiguration('jetson_ip'),
                       ':', LaunchConfiguration('bottom_port'), '/video\n',
        '║                                                                  ║\n',
        '║  Verify mavlink-router:  systemctl status mavlink-router         ║\n',
        '╚══════════════════════════════════════════════════════════════════╝',
    ])

    # ═══════════════════════════════════════════════════════════════════════
    #  Launch Description
    # ═══════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        # ── MAVROS args ──
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        system_id_arg,
        component_id_arg,
        namespace_arg,
        # ── Bridge args ──
        jetson_ip_arg,
        gcs_ip_arg,
        cmd_port_arg,
        telem_port_arg,
        # ── Webcam args ──
        enable_webcam_arg,
        front_dev_arg,
        bottom_dev_arg,
        front_port_arg,
        bottom_port_arg,
        cam_width_arg,
        cam_height_arg,
        cam_fps_arg,
        # ── Banner ──
        banner,
        # ── Nodes ──
        mavros_node,
        gcs_bridge_node,
        webcam_node,
    ])
