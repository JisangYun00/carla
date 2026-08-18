// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

// HMC vehicle geometry configuration.
// Field order/count matches ads_interfaces/msg/VehicleConfiguration.msg.
// Published once per hero vehicle by CARLA (transient-local semantics).

#pragma once
#include "Header.h"
#include <cstdint>

namespace carla {
namespace ros2 {
namespace msg {

struct HmcVehicleConfig {
  Header header {};
  float vehicle_width_m {0.0f};
  float vehicle_length_m {0.0f};
  uint8_t ignition_default_on {1u};
};

} // namespace msg
} // namespace ros2
} // namespace carla