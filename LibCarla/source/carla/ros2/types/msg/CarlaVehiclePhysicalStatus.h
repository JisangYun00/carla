// Copyright (c) 2026 Hanyang University.
// Developed by Automotive Intelligence Lab.
// SPDX-License-Identifier: MIT

/**
 * Module:      CarlaVehiclePhysicalStatus.h
 * Description: Native POD for carla_msgs::msg::CarlaVehiclePhysicalStatus.
 *              Mirrors the canonical .msg definition in
 *              Util/ros2/carla_msgs/msg/CarlaVehiclePhysicalStatus.msg.
 *              All numeric fields use the exact ROS 2 IDL primitive types so
 *              that Fast-CDR serialization is byte-compatible with generated
 *              ROS 2 bindings.
 *
 * Authors:
 *   Jisang Yun (jisangyun@hanyang.ac.kr)
 *
 * Revision History:
 *   2026-08-25: Jisang Yun - Created.
 */

#pragma once

#include <cstdint>

#include "carla/ros2/types/msg/Header.h"

namespace carla {
namespace ros2 {
namespace msg {

struct CarlaVehiclePhysicalStatus {
  Header header {};
  bool brake_status = false;
  bool abs_status = false;
  bool tcs_status = false;
  bool esc_status = false;
  uint8_t current_gear = 0u;
  float yaw_rate_radps = 0.0f;
  float lateral_acceleration_mps2 = 0.0f;
  float longitudinal_acceleration_mps2 = 0.0f;
  float steering_wheel_angle_deg = 0.0f;
  float vehicle_width_m = 0.0f;
  float vehicle_length_m = 0.0f;
  float vehicle_speed_kmh = 0.0f;
  float wheel_angular_velocity_fl_radps = 0.0f;
  float wheel_angular_velocity_fr_radps = 0.0f;
  float wheel_angular_velocity_rl_radps = 0.0f;
  float wheel_angular_velocity_rr_radps = 0.0f;
  bool ignition_status = false;
  uint64_t valid_fields = 0u;
};

} // namespace msg
} // namespace ros2
} // namespace carla
