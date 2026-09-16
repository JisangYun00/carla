// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace carla {
namespace ros2 {

// Models the steering-wheel position sensor that CARLA does not expose. The
// delay is the measured native HMC command-to-wheel response (100..600 deg).
class HmcVirtualSteeringActuator {
 public:
  static constexpr double kDefaultResponseDelaySec = 0.15;

  void SetResponseDelaySec(double response_delay_sec) {
    if (std::isfinite(response_delay_sec) && response_delay_sec >= 0.0) {
      _response_delay_sec = response_delay_sec;
    }
  }

  void ObserveTarget(float target_swa_deg, double received_at_sec) {
    if (!std::isfinite(target_swa_deg) || !std::isfinite(received_at_sec)) return;
    if (_count == _samples.size()) {
      _head = (_head + 1) % _samples.size();
      --_count;
    }
    const std::size_t tail = (_head + _count) % _samples.size();
    _samples[tail] = {target_swa_deg, received_at_sec};
    ++_count;
  }

  void Advance(double now_sec) {
    if (!std::isfinite(now_sec)) return;
    const double cutoff = now_sec - _response_delay_sec;
    while (_count > 0 && _samples[_head].received_at_sec <= cutoff + 1e-9) {
      _actual_swa_deg = _samples[_head].target_swa_deg;
      _head = (_head + 1) % _samples.size();
      --_count;
    }
  }

  float GetActualSwaDeg() const { return _actual_swa_deg; }

 private:
  struct Sample {
    float target_swa_deg;
    double received_at_sec;
  };

  // 64 samples retain 640 ms at the 100 Hz AD-01 cadence.
  std::array<Sample, 64> _samples{};
  std::size_t _head{0};
  std::size_t _count{0};
  float _actual_swa_deg{0.0f};
  double _response_delay_sec{kDefaultResponseDelaySec};
};

}  // namespace ros2
}  // namespace carla
