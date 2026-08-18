// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

// HMC vehicle control-unit status.
// Field order/count matches ads_interfaces/msg/VehicleControlUnitState.msg.
// Published natively by CARLA so the ADS stack needs no external provider.

#pragma once
#include "Header.h"
#include <cstdint>

namespace carla {
namespace ros2 {
namespace msg {

struct HmcVehicleStatus {
  Header header {};
  uint8_t current_gear {0u};
  uint8_t brake_status {0u};
  uint8_t abs_status {0u};
  uint8_t tcs_status {0u};
  uint8_t esc_status {0u};
  float vehicle_speed_kmh {0.0f};
  float actual_swa_deg {0.0f};
  uint8_t ignition_on {1u};
  uint64_t valid_flags {0u};
};

} // namespace msg
} // namespace ros2
} // namespace carla