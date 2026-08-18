// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

// HMC AD-02 emergency / fail-safe control command.
// Field order/count matches hmc_interfaces/msg/AD02.msg.

#pragma once
#include <cstdint>

namespace carla {
namespace ros2 {
namespace msg {

struct HmcAD02 {
  uint16_t crc = 0u;
  uint8_t alive_cnt = 0u;
  uint8_t emgc_brk_active = 0u;
  uint8_t emgc_steer_active = 0u;
  uint8_t emgc_brk_stop_hold = 0u;
  uint8_t emgc_brk_mode = 0u;
  uint8_t emgc_decel_tgt_g = 0u;
  int16_t emgc_steer_ang_tgt_deg = 0;
  uint8_t emgc_accel_tgt_g = 0u;
  uint8_t emgc_accel_mode = 0u;
  uint8_t emgc_accel_active = 0u;
  uint8_t emgc_reason_code = 0u;
  uint8_t driver_takeover_req = 0u;
};

} // namespace msg
} // namespace ros2
} // namespace carla
