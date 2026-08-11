// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "HmcVehicleStatusPublisher.h"

namespace carla {
namespace ros2 {

bool HmcVehicleStatusPublisher::Write(
    int32_t stamp_sec,
    uint32_t stamp_nanosec,
    const std::string& frame_id,
    uint8_t current_gear,
    uint8_t brake_status,
    float vehicle_speed_kmh,
    float actual_swa_deg,
    uint8_t ignition_on,
    uint64_t valid_flags) {
  auto* msg = _impl->GetMessage();
  if (!msg) return false;
  msg->header.stamp.sec = stamp_sec;
  msg->header.stamp.nanosec = stamp_nanosec;
  msg->header.frame_id = frame_id;
  msg->current_gear = current_gear;
  msg->brake_status = brake_status;
  msg->abs_status = 0u;      // [VERIFY] CARLA does not expose ABS
  msg->tcs_status = 0u;      // [VERIFY] CARLA does not expose TCS
  msg->esc_status = 0u;      // [VERIFY] CARLA does not expose ESC
  msg->vehicle_speed_kmh = vehicle_speed_kmh;
  msg->actual_swa_deg = actual_swa_deg;
  msg->ignition_on = ignition_on;
  msg->valid_flags = valid_flags;
  return true;
}

}  // namespace ros2
}  // namespace carla