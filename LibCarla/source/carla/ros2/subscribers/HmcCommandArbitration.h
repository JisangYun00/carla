// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

#pragma once

namespace carla {
namespace ros2 {

  enum class HmcCommandSource { AD01, AD02, Timeout };

  template <typename Age, typename Freshness>
  constexpr HmcCommandSource SelectHmcCommand(
      bool ad01_fresh,
      bool ad02_brake_active,
      bool ad02_steer_active,
      Age ad02_age,
      Freshness ad02_freshness) {
    return (ad02_brake_active || ad02_steer_active) && ad02_age < ad02_freshness
        ? HmcCommandSource::AD02
        : ad01_fresh ? HmcCommandSource::AD01 : HmcCommandSource::Timeout;
  }

}  // namespace ros2
}  // namespace carla
