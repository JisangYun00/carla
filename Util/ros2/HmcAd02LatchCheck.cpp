// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

#include <cassert>
#include <chrono>

#include "carla/ros2/subscribers/HmcCommandArbitration.h"

int main() {
  using namespace std::chrono;
  using carla::ros2::HmcCommandSource;
  using carla::ros2::SelectHmcCommand;

  const auto freshness = milliseconds(30);

  assert(SelectHmcCommand(false, true, false, milliseconds(0), freshness) ==
      HmcCommandSource::AD02);
  assert(SelectHmcCommand(true, true, false, milliseconds(29), freshness) ==
      HmcCommandSource::AD02);
  assert(SelectHmcCommand(true, false, true, milliseconds(29), freshness) ==
      HmcCommandSource::AD02);
  assert(SelectHmcCommand(true, false, false, milliseconds(1), freshness) ==
      HmcCommandSource::AD01);
  assert(SelectHmcCommand(true, true, false, milliseconds(30), freshness) ==
      HmcCommandSource::AD01);
  assert(SelectHmcCommand(false, false, false, milliseconds(1), freshness) ==
      HmcCommandSource::Timeout);
  assert(SelectHmcCommand(false, true, false, milliseconds(30), freshness) ==
      HmcCommandSource::Timeout);
}
