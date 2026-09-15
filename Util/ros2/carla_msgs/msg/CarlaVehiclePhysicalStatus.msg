#
# Copyright (c) 2026 Hanyang University.
# Developed by Automotive Intelligence Lab.
# SPDX-License-Identifier: MIT
#
# Author: Jisang Yun <jisangyun@hanyang.ac.kr>
#
# CARLA Sensor/Status Provider atomic physical status snapshot.
#
# This message aggregates vehicle physical state and geometry that the ADS
# stack receives through a single external provider interface. All fields are
# in physical units; per-signal validity is reported in valid_signals.
#
# See project_docs/03_인터페이스명세.md §3.5 (D24).

std_msgs/Header header

# Chassis/ brake system active flags. Unsupported features must be published as
# false with the corresponding valid_signals bit cleared.
bool brake_status
bool abs_status
bool tcs_status
bool esc_status

# Current transmission gear.
# 1 = Park, 2 = Reverse, 3 = Neutral, 5 = Drive.
# Publish with the valid_signals bit cleared if CARLA cannot distinguish it.
uint8 current_gear

# Vehicle vertical-axis yaw rate, rad/s.
float32 yaw_rate_radps

# Vehicle coordinate-frame accelerations, m/s^2.
float32 lateral_acceleration_mps2
float32 longitudinal_acceleration_mps2

# Actual steering wheel angle after steering ratio, degrees.
# This is NOT the normalized control.steer value; it is the physical SWA.
float32 steering_wheel_angle_deg

# Vehicle bounding-box dimensions in meters.
float32 vehicle_width_m
float32 vehicle_length_m

# Actual vehicle speed magnitude, km/h.
float32 vehicle_speed_kmh

# Ignition ON/OFF. Publish only when the simulator has an explicit state;
# otherwise clear the corresponding valid_signals bit.
bool ignition_status

# Per-signal validity bitmap (CARLA provider contract).
# Bit assignment (bit set = field valid):
#   0 : brake_status
#   1 : abs_status
#   2 : tcs_status
#   3 : esc_status
#   4 : current_gear
#   5 : yaw_rate_radps
#   6 : lateral_acceleration_mps2
#   7 : longitudinal_acceleration_mps2
#   8 : steering_wheel_angle_deg
#   9 : vehicle_width_m
#  10 : vehicle_length_m
#  11 : vehicle_speed_kmh
#  12 : ignition_status
#  13..63 : reserved, set to 0
uint64 valid_signals
