# Copyright 2026 ICAD — RYUGU ROV Project
# Licensed under the MIT License.

"""
ryugu_control — ROS2 control package for the RYUGU underwater ROV.

This package provides nodes and launch configurations for controlling
the RYUGU ROV via MAVROS, communicating with a Pixhawk 2.4.8 running
ArduSub v4.5.7.

Hardware:
  - 6 Thrusters (DSHOT ESCs) on Pixhawk AUX 1–6
  - 1 Gripper Servo (MG996R) on Pixhawk MAIN 1
  - Jetson Orin Nano ↔ Pixhawk via USB serial (/dev/ttyACM0)
"""
