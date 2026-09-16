#!/usr/bin/env bash
#
# Copyright (c) 2026 Hanyang University.
# Developed by Automotive Intelligence Lab.
# SPDX-License-Identifier: MIT
#
# Author: Jisang Yun <jisangyun@hanyang.ac.kr>
#
# Build the carla_msgs package and verify that CarlaVehiclePhysicalStatus
# is generated and can be introspected with `ros2 interface show`.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${WS_DIR}"
}
trap cleanup EXIT

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon not found. Source a ROS 2 environment first."
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 CLI not found. Source a ROS 2 environment first."
  exit 1
fi

mkdir -p "${WS_DIR}/src"
cp -a "${PKG_DIR}" "${WS_DIR}/src/carla_msgs"

echo "==> Building carla_msgs in ${WS_DIR}"
cd "${WS_DIR}"
colcon build --packages-select carla_msgs --cmake-args -DBUILD_TESTING=OFF

echo "==> Sourcing install"
set +u
source "${WS_DIR}/install/setup.bash"
set -u

echo "==> ros2 interface show carla_msgs/msg/CarlaVehiclePhysicalStatus"
ros2 interface show carla_msgs/msg/CarlaVehiclePhysicalStatus

echo "==> Verifying expected fields"
INTERFACE_OUTPUT="$(ros2 interface show carla_msgs/msg/CarlaVehiclePhysicalStatus)"
for field in \
  "std_msgs/Header header" \
  "bool brake_status" \
  "bool abs_status" \
  "bool tcs_status" \
  "bool esc_status" \
  "uint8 current_gear" \
  "float32 yaw_rate_radps" \
  "float32 lateral_acceleration_mps2" \
  "float32 longitudinal_acceleration_mps2" \
  "float32 steering_wheel_angle_deg" \
  "float32 vehicle_width_m" \
  "float32 vehicle_length_m" \
  "float32 vehicle_speed_kmh" \
  "float32 wheel_angular_velocity_fl_radps" \
  "float32 wheel_angular_velocity_fr_radps" \
  "float32 wheel_angular_velocity_rl_radps" \
  "float32 wheel_angular_velocity_rr_radps" \
  "bool ignition_status" \
  "uint64 valid_fields"; do
  if ! grep -qF "${field}" <<<"${INTERFACE_OUTPUT}"; then
    echo "ERROR: expected field missing: ${field}"
    exit 1
  fi
done

echo "==> ros2 interface list | grep CarlaVehiclePhysicalStatus"
ros2 interface list | grep CarlaVehiclePhysicalStatus

echo "==> Verifying RespawnVehicle service"
RESPAWN_OUTPUT="$(ros2 interface show carla_msgs/srv/RespawnVehicle)"
for field in \
  "uint8 location_mode" \
  "float64 latitude" \
  "float64 longitude" \
  "float64 heading_deg" \
  "bool success" \
  "string message" \
  "uint32 hero_id" \
  "uint32 gnss_id" \
  "uint32 imu_id"; do
  if ! grep -qF "${field}" <<<"${RESPAWN_OUTPUT}"; then
    echo "ERROR: expected RespawnVehicle field missing: ${field}"
    exit 1
  fi
done

echo "==> PASS"
