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
# in physical units; per-signal validity is reported in valid_fields.
#
# See project_docs/03_인터페이스명세.md §3.5 (D24).

std_msgs/Header header

# Chassis/ brake system active flags. Unsupported features must be published as
# false with the corresponding valid_fields bit cleared.
bool brake_status
bool abs_status
bool tcs_status
bool esc_status

# Current transmission gear.
# 1 = Park, 2 = Reverse, 3 = Neutral, 5 = Drive.
# Publish with the valid_fields bit cleared if CARLA cannot distinguish it.
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

# Four-wheel angular speeds from PhysX, rad/s. Positive is forward rotation.
float32 wheel_angular_velocity_fl_radps
float32 wheel_angular_velocity_fr_radps
float32 wheel_angular_velocity_rl_radps
float32 wheel_angular_velocity_rr_radps

# Ignition ON/OFF. Publish only when the simulator has an explicit state;
# otherwise clear the corresponding valid_fields bit.
bool ignition_status

# Per-field validity bitmap. Bit 0 starts at brake_status in this declaration;
# header and valid_fields are excluded.
uint64 valid_fields
