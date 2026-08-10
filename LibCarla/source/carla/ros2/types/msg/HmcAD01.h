// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// HMC AD-01 normal autonomous driving motion control command.
// Field order/count matches hmc_interfaces/msg/AD01.msg.

#pragma once
#include <cstdint>

namespace carla {
namespace ros2 {
namespace msg {

struct HmcAD01 {
  uint16_t crc = 0u;
  uint8_t alive_cnt = 0u;
  uint8_t lng_ctrl_engage_req = 0u;
  uint8_t lat_ctrl_engage_req = 0u;
  uint8_t lng_ctrl_type = 0u;
  uint8_t lat_ctrl_type = 0u;
  uint8_t target_gear = 0u;
  uint16_t target_aps_pct = 0u;
  uint16_t target_bps_pct = 0u;
  uint8_t stop_hold_req = 0u;
  int16_t target_speed_kmh = 0;
  int16_t target_wheel_tq_nm = 0;
  int16_t target_swa_deg = 0;
  int16_t target_steer_tq_nm = 0;
};

} // namespace msg
} // namespace ros2
} // namespace carla
