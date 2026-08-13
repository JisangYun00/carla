// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "HmcFeedbackPublisher.h"

#include <cstdint>
#include <algorithm>
#include <cmath>

namespace carla {
namespace ros2 {

namespace {
  constexpr float kPhysicalToRaw = 10.0f;
  constexpr float kMaxPctRaw = 1023.0f;

  inline uint16_t saturate_pct(float pct) {
    const float raw = pct * kPhysicalToRaw;  // HMC factor: 0.1 percent
    if (raw < 0.0f) return 0u;
    if (raw > kMaxPctRaw) return static_cast<uint16_t>(kMaxPctRaw);
    return static_cast<uint16_t>(raw);
  }

  inline int16_t saturate_int16(float physical) {
    const float raw = physical * kPhysicalToRaw;  // HMC factor: 0.1
    if (raw < -32768.0f) return -32768;
    if (raw > 32767.0f) return 32767;
    return static_cast<int16_t>(raw);
  }

  inline uint8_t clamp_ready(uint8_t value) {
    return (value == 0u || value == 1u) ? value : 1u;
  }
}

bool HmcFeedbackPublisher::Write(
    uint8_t alive_counter,
    float aps_pct,
    float bps_pct,
    float actual_speed_kmh,
    float target_speed_echo_kmh,
    float actual_swa_deg,
    float target_swa_echo_deg,
    uint8_t lng_op_mode,
    uint8_t lat_op_mode,
    bool actuator_fault,
    uint8_t lng_ctrl_ready,
    uint8_t lat_ctrl_ready,
    uint8_t gear_sel_ready) {
  auto* msg = _impl->GetMessage();
  if (!msg) {
    return false;
  }

  msg->crc = 0u;  // [VERIFY] CRC policy for FB-01
  msg->alive_cnt = alive_counter;
  msg->lng_ctrl_ready = clamp_ready(lng_ctrl_ready);
  msg->lat_ctrl_ready = clamp_ready(lat_ctrl_ready);
  msg->lng_ctrl_type_active = 0u;  // [VERIFY] active control type mapping
  msg->lat_ctrl_type_active = 0u;
  msg->gear_sel_ready = clamp_ready(gear_sel_ready);
  msg->stop_hold_ready = 0u;
  msg->actuator_fault_sta = actuator_fault ? 1u : 0u;
  msg->lng_op_mode = lng_op_mode;
  msg->lat_op_mode = lat_op_mode;
  msg->aps_fdb_pct = saturate_pct(aps_pct);
  msg->bps_fdb_pct = saturate_pct(bps_pct);
  msg->actual_speed_kmh = saturate_int16(actual_speed_kmh);
  msg->target_speed_echo_kmh = saturate_int16(target_speed_echo_kmh);
  msg->wheel_tq_fdb_nm = 0;  // [VERIFY] not available from CARLA
  msg->actual_swa_deg = saturate_int16(actual_swa_deg);
  msg->target_swa_echo_deg = saturate_int16(target_swa_echo_deg);

  return true;
}

}  // namespace ros2
}  // namespace carla
