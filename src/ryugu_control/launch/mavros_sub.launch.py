"""
MAVROS Launch File for RYUGU ROV — ArduSub v4.5.7

Launches the MAVROS node configured for a Pixhawk 2.4.8 connected via
USB serial (/dev/ttyACM0) at 115200 baud.

Hardware Map:
  AUX 1–6  → 6 Thrusters (DSHOT ESCs)
  MAIN 1   → Gripper Servo (MG996R)

Usage:
  ros2 launch ryugu_control mavros_sub.launch.py

  # Override serial port:
  ros2 launch ryugu_control mavros_sub.launch.py fcu_url:=/dev/ttyUSB0:115200

  # Connect via UDP (e.g., from MAVProxy/SITL):
  ros2 launch ryugu_control mavros_sub.launch.py fcu_url:=udp://:14540@
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros_share = get_package_share_directory('mavros')
    config_yaml = os.path.join(mavros_share, 'launch', 'apm_config.yaml')
    pluginlists_yaml = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')

    # ── Declare launch arguments ────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:115200',
        description=(
            'FCU connection URL.  '
            'Serial example : /dev/ttyACM0:115200  '
            'Fallback serial: /dev/ttyUSB0:115200  '
            'UDP example    : udp://:14540@'
        ),
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='GCS bridge URL (leave empty to disable)',
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='MAVLink target system ID (Pixhawk default = 1)',
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='MAVLink target component ID',
    )

    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='1',
        description='MAVLink system ID for this MAVROS instance',
    )

    component_id_arg = DeclareLaunchArgument(
        'component_id',
        default_value='240',
        description='MAVLink component ID for this MAVROS instance (240 = GCS)',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='mavros',
        description='ROS2 namespace for the MAVROS node',
    )

    # ── MAVROS Node ─────────────────────────────────────────────────────
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

                # ── Plug-in denylist (disable unused plugins to save CPU) ──
                'plugin_denylist': [
                    'global_position',
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
                'conn/heartbeat_rate': 1.0,   # Hz
                'conn/timeout':        10.0,  # seconds
                'conn/timesync_rate':  10.0,  # Hz

                # ── IMU ──
                'imu/frame_id':                  'base_link',
                'imu/linear_acceleration_stdev': 0.0003,
                'imu/angular_velocity_stdev':    0.0003,
                'imu/orientation_stdev':         1.0,

                # ── Local position ──
                'local_position/frame_id': 'map',
                'local_position/tf/send':   True,
                'local_position/tf/frame_id':       'map',
                'local_position/tf/child_frame_id': 'base_link',

                # ── Global position ──
                'global_position/frame_id':        'map',
                'global_position/child_frame_id':  'base_link',
                'global_position/tf/send':          False,

                # ── Setpoint attitude ──
                'setpoint_attitude/reverse_thrust': True,  # ArduSub supports reverse thrust

                # ── System ──
                'sys/min_voltage':   10.0,    # Low-voltage warning threshold (V)
                'sys/disable_diag':  False,
            }
        ],
    )

    # ── Banner ──────────────────────────────────────────────────────────
    banner = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════════════════╗\n',
        '║           RYUGU ROV — MAVROS ArduSub v4.5.7                   \n',
        '║  FCU URL : ', LaunchConfiguration('fcu_url'), '\n',
        '║  SYS  ID : ', LaunchConfiguration('system_id'), '\n',
        '║  COMP ID : ', LaunchConfiguration('component_id'), '\n',
        '╚══════════════════════════════════════════════════════════════╝',
    ])

    return LaunchDescription([
        # Arguments
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        system_id_arg,
        component_id_arg,
        namespace_arg,
        # Log
        banner,
        # Nodes
        mavros_node,
    ])
