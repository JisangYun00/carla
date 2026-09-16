// Copyright (c) 2026 Hanyang University.
// Developed by Automotive Intelligence Lab.
// SPDX-License-Identifier: MIT
//
/**
 * Module:      CarlaVehiclePhysicalStatusPublisher.cpp
 * Description: Implementation of the atomic physical status publisher Write()
 *              method. All fields are written into the held message in one
 *              call so the header timestamp is identical for every signal.
 *
 * Authors:
 *   Jisang Yun (jisangyun@hanyang.ac.kr)
 *
 * Revision History:
 *   2026-08-25: Jisang Yun - Created.
 */

#include "CarlaVehiclePhysicalStatusPublisher.h"

namespace carla {
namespace ros2 {

bool CarlaVehiclePhysicalStatusPublisher::Write(
    int32_t stamp_sec,
    uint32_t stamp_nanosec,
    const std::string& frame_id,
    bool brake_status,
    bool abs_status,
    bool tcs_status,
    bool esc_status,
    uint8_t current_gear,
    float yaw_rate_radps,
    float lateral_acceleration_mps2,
    float longitudinal_acceleration_mps2,
    float steering_wheel_angle_deg,
    float vehicle_width_m,
    float vehicle_length_m,
    float vehicle_speed_kmh,
    float wheel_angular_velocity_fl_radps,
    float wheel_angular_velocity_fr_radps,
    float wheel_angular_velocity_rl_radps,
    float wheel_angular_velocity_rr_radps,
    bool ignition_status,
    uint64_t valid_fields) {
  auto* msg = _impl->GetMessage();
  if (!msg) return false;
  msg->header.stamp.sec = stamp_sec;
  msg->header.stamp.nanosec = stamp_nanosec;
  msg->header.frame_id = frame_id;
  msg->brake_status = brake_status;
  msg->abs_status = abs_status;
  msg->tcs_status = tcs_status;
  msg->esc_status = esc_status;
  msg->current_gear = current_gear;
  msg->yaw_rate_radps = yaw_rate_radps;
  msg->lateral_acceleration_mps2 = lateral_acceleration_mps2;
  msg->longitudinal_acceleration_mps2 = longitudinal_acceleration_mps2;
  msg->steering_wheel_angle_deg = steering_wheel_angle_deg;
  msg->vehicle_width_m = vehicle_width_m;
  msg->vehicle_length_m = vehicle_length_m;
  msg->vehicle_speed_kmh = vehicle_speed_kmh;
  msg->wheel_angular_velocity_fl_radps = wheel_angular_velocity_fl_radps;
  msg->wheel_angular_velocity_fr_radps = wheel_angular_velocity_fr_radps;
  msg->wheel_angular_velocity_rl_radps = wheel_angular_velocity_rl_radps;
  msg->wheel_angular_velocity_rr_radps = wheel_angular_velocity_rr_radps;
  msg->ignition_status = ignition_status;
  msg->valid_fields = valid_fields & 0x1FFFFu;
  return true;
}

}  // namespace ros2
}  // namespace carla
