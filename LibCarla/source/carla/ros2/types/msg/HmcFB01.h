// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// HMC FB-01 motion control status feedback.
// Field order/count matches hmc_interfaces/msg/FB01.msg.

#pragma once
#include <cstdint>

namespace carla {
namespace ros2 {
namespace msg {

struct HmcFB01 {
  uint16_t crc = 0u;
  uint8_t alive_cnt = 0u;
  uint8_t lng_ctrl_ready = 0u;
  uint8_t lat_ctrl_ready = 0u;
  uint8_t lng_ctrl_type_active = 0u;
  uint8_t lat_ctrl_type_active = 0u;
  uint8_t gear_sel_ready = 0u;
  uint8_t stop_hold_ready = 0u;
  uint8_t actuator_fault_sta = 0u;
  uint8_t lng_op_mode = 0u;
  uint8_t lat_op_mode = 0u;
  uint16_t aps_fdb_pct = 0u;
  uint16_t bps_fdb_pct = 0u;
  int16_t actual_speed_kmh = 0;
  int16_t target_speed_echo_kmh = 0;
  int16_t wheel_tq_fdb_nm = 0;
  int16_t actual_swa_deg = 0;
  int16_t target_swa_echo_deg = 0;
};

} // namespace msg
} // namespace ros2
} // namespace carla
