#!/usr/bin/env python3
"""
qgc_signal_monitor.py — ROS 2 QGroundControl Signal & Hardware Telemetry Monitor

Purpose
-------
Passively monitor and display all physical motor outputs (AUX 1–6), Gripper servo
outputs (MAIN 1), vehicle mode, and manual control setpoints sent by QGroundControl
in real-time via MAVROS and mavlink-router.

This node is PASSIVE (telemetry-only). It does not publish control commands,
allowing QGroundControl to maintain full, uninhibited control over the ROV.

Topics Subscribed
-----------------
  • /mavros/state                  (mavros_msgs/msg/State)         — FCU connection, arm state, mode
  • /mavros/rc/out                 (mavros_msgs/msg/RCOut)         — Physical PWM / DSHOT output array
  • /mavros/manual_control/control (mavros_msgs/msg/ManualControl)  — Manual control commands from QGC
  • /mavros/battery                (sensor_msgs/msg/BatteryState)  — Battery voltage

Usage
-----
  1. Ensure mavlink-router is active:
     sudo systemctl start mavlink-router

  2. Launch MAVROS in isolation:
     ros2 launch ryugu_control mavros_sub.launch.py

  3. Connect QGroundControl to the vehicle.

  4. Run this monitoring node:
     ros2 run ryugu_control qgc_signal_monitor
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State, RCOut, ManualControl
from sensor_msgs.msg import BatteryState


class C:
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    DIM     = '\033[2m'
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    CYAN    = '\033[96m'
    WHITE   = '\033[97m'


class QGCSignalMonitorNode(Node):
    """
    ROS 2 Telemetry & Signal Monitoring Node for QGroundControl sessions.
    """

    def __init__(self):
        super().__init__('qgc_signal_monitor')
        self.get_logger().info('Initializing QGCSignalMonitorNode (Passive Telemetry)...')

        # ── State Variables ───────────────────────────────────────────
        self._fcu_connected = False
        self._armed = False
        self._mode = 'UNKNOWN'
        self._batt_v = 0.0

        # RCOut channels (16 channels)
        self._rc_out = [0] * 16

        # QGC Manual Control setpoints
        self._mc_x = 0.0
        self._mc_y = 0.0
        self._mc_z = 0.0
        self._mc_r = 0.0
        self._mc_buttons = 0
        self._last_mc_time = 0.0

        # Event tracking
        self._last_mode = None
        self._last_armed = None

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            State,
            '/mavros/state',
            self._state_cb,
            10)

        self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self._rc_out_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            ManualControl,
            '/mavros/manual_control/control',
            self._manual_control_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_cb,
            qos_profile_sensor_data)

        # ── 5 Hz Status HUD Timer ─────────────────────────────────────
        self._hud_timer = self.create_timer(0.2, self._draw_hud)

        self._print_banner()

    def _print_banner(self):
        print(f'{C.BOLD}{"═" * 72}{C.RESET}')
        print(f'{C.BOLD}{C.CYAN}    RYUGU ROV — QGroundControl Signal & Hardware Telemetry Monitor{C.RESET}')
        print(f'{C.BOLD}{"═" * 72}{C.RESET}')
        print(f'  Mode:       {C.GREEN}PASSIVE (Telemetry-Only — QGC has full control){C.RESET}')
        print(f'  Monitors:   Mode, Arm State, Battery, QGC ManualControl, Servo & Thrusters')
        print(f'  Thrusters:  AUX 1–6 (Motor PWM / DSHOT outputs)')
        print(f'  Gripper:    MAIN 1 (Servo 1 PWM output)')
        print(f'{C.BOLD}{"─" * 72}{C.RESET}')
        print(f'  Press {C.BOLD}Ctrl+C{C.RESET} to exit monitor.')
        print(f'{C.BOLD}{"═" * 72}{C.RESET}\n')

    # ── Subscriber Callbacks ───────────────────────────────────────────
    def _state_cb(self, msg: State):
        self._fcu_connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode if msg.mode else 'UNKNOWN'

        # Log mode/arm changes as discrete events
        if self._last_armed is not None and self._last_armed != self._armed:
            st = f'{C.GREEN}ARMED{C.RESET}' if self._armed else f'{C.RED}DISARMED{C.RESET}'
            self._log_event(f'Vehicle Arm State Changed → {st}')
        self._last_armed = self._armed

        if self._last_mode is not None and self._last_mode != self._mode:
            self._log_event(f'Vehicle Mode Changed → {C.CYAN}{self._mode}{C.RESET}')
        self._last_mode = self._mode

    def _rc_out_cb(self, msg: RCOut):
        if msg.channels:
            chans = list(msg.channels)
            if len(chans) < 16:
                chans.extend([0] * (16 - len(chans)))
            self._rc_out = chans[:16]

    def _manual_control_cb(self, msg: ManualControl):
        self._mc_x = msg.x
        self._mc_y = msg.y
        self._mc_z = msg.z
        self._mc_r = msg.r
        self._mc_buttons = msg.buttons
        self._last_mc_time = time.time()

    def _battery_cb(self, msg: BatteryState):
        if msg.voltage > 0:
            self._batt_v = msg.voltage

    # ── Console Output ─────────────────────────────────────────────────
    def _log_event(self, text):
        sys.stdout.write('\r\x1b[K' + f'[EVENT] {text}\n')
        sys.stdout.flush()

    def _draw_hud(self):
        arm_str = f'{C.RED}{C.BOLD}ARMED{C.RESET}' if self._armed else f'{C.GREEN}DISARMED{C.RESET}'
        conn_str = f'{C.GREEN}FCU OK{C.RESET}' if self._fcu_connected else f'{C.RED}FCU OFF{C.RESET}'
        mode_str = f'{C.CYAN}[{self._mode}]{C.RESET}'

        # Extract Gripper (MAIN 1 = channel 1) and Thrusters (AUX 1-6 = channels 9-14)
        main1_pwm = self._rc_out[0]
        aux_pwms = self._rc_out[8:14]
        aux_str = ' '.join(f'{v:>4}' for v in aux_pwms)

        # Gripper state estimation based on PWM width
        if main1_pwm <= 1350 and main1_pwm > 0:
            grip_label = f'{C.GREEN}OPEN{C.RESET}'
        elif main1_pwm >= 1900:
            grip_label = f'{C.YELLOW}CLOSE{C.RESET}'
        elif main1_pwm > 0:
            grip_label = f'{C.DIM}MID{C.RESET}'
        else:
            grip_label = f'{C.DIM}OFF{C.RESET}'

        # QGC Manual Control active check (< 1.5s old)
        qgc_active = (time.time() - self._last_mc_time) < 1.5
        qgc_str = f'{C.GREEN}QGC ACTIVE{C.RESET}' if qgc_active else f'{C.DIM}NO QGC{C.RESET}'

        hud_line = (
            f'{mode_str} {arm_str} {conn_str} {self._batt_v:5.1f}V | '
            f'QGC({qgc_str}): x={int(self._mc_x):+04d} y={int(self._mc_y):+04d} z={int(self._mc_z):+04d} r={int(self._mc_r):+04d} | '
            f'MAIN1:{main1_pwm:>4}µs({grip_label}) | '
            f'AUX[1-6]: {aux_str}'
        )

        sys.stdout.write('\r\x1b[K' + hud_line)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = QGCSignalMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f'\n{C.YELLOW}QGC Signal Monitor stopped.{C.RESET}')


if __name__ == '__main__':
    main()
