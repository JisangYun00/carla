// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

#include "HmcVehicleConfigPublisher.h"

namespace carla {
namespace ros2 {

bool HmcVehicleConfigPublisher::Write(
    int32_t stamp_sec,
    uint32_t stamp_nanosec,
    const std::string& frame_id,
    float vehicle_width_m,
    float vehicle_length_m,
    uint8_t ignition_default_on) {
  auto* msg = _impl->GetMessage();
  if (!msg) return false;
  msg->header.stamp.sec = stamp_sec;
  msg->header.stamp.nanosec = stamp_nanosec;
  msg->header.frame_id = frame_id;
  msg->vehicle_width_m = vehicle_width_m;
  msg->vehicle_length_m = vehicle_length_m;
  msg->ignition_default_on = ignition_default_on;
  return true;
}

}  // namespace ros2
}  // namespace carla