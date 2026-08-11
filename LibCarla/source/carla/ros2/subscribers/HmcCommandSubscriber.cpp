// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "HmcCommandSubscriber.h"

#include <chrono>

#include "carla/ros2/ROS2CallbackData.h"

namespace carla {
namespace ros2 {

  // Conversion helpers from HMC raw/physical units to CARLA VehicleControl.
  // [VERIFY] scale factors match vehicle-specific APS/BPS calibration.
  namespace {
    constexpr float kMaxSteerAngleDeg = 540.0f;
    constexpr float kRawToPhysical = 0.1f;

    inline float deg_to_steer_ratio(int16_t raw_deg) {
      // AD-01 SWA has a 0.1 deg factor; CARLA steer is full-lock [-1, 1].
      const float ratio = raw_deg * kRawToPhysical / kMaxSteerAngleDeg;
      return std::min(1.0f, std::max(-1.0f, ratio));
    }

    inline float pct_to_throttle(uint16_t raw_pct) {
      // AD-01 APS/BPS has a 0.1 percent factor.
      return std::min(1.0f, raw_pct * kRawToPhysical / 100.0f);
    }
  }

  ROS2CallbackData HmcCommandSubscriber::GetMessage() {
    const bool ad01_new = _ad01_impl->HasNewMessage();
    const bool ad02_new = _ad02_impl->HasNewMessage();

    if (ad01_new || ad02_new) {
      _last_command_time = std::chrono::steady_clock::now();
    }

    auto ad01 = ad01_new ? _ad01_impl->GetMessage() : _latest_ad01;
    auto ad02 = ad02_new ? _ad02_impl->GetMessage() : _latest_ad02;

    // Cache latest values so AD-01/02 arriving at different times still merge.
    if (ad01_new) _latest_ad01 = ad01;
    if (ad02_new) _latest_ad02 = ad02;

    VehicleControl control{};

    // Command timeout: fail-safe brake if no fresh AD-01/AD-02.
    if (!HasFreshCommand()) {
      control.brake = 1.0f;
      control.throttle = 0.0f;
      control.steer = 0.0f;
      return control;
    }

    // Normal command (AD-01)
    if (ad01.lat_ctrl_engage_req) {
      control.steer = deg_to_steer_ratio(ad01.target_swa_deg);
    }
    if (ad01.lng_ctrl_engage_req) {
      control.throttle = pct_to_throttle(ad01.target_aps_pct);
      control.brake = pct_to_throttle(ad01.target_bps_pct);
    }

    // Emergency override (AD-02) takes precedence.
    if (ad02.emgc_brk_active) {
      control.brake = 1.0f;
      control.throttle = 0.0f;
    }
    if (ad02.emgc_steer_active) {
      control.steer = deg_to_steer_ratio(ad02.emgc_steer_ang_tgt_deg);
    }

    // Map HMC gear enum to CARLA VehicleControl fields.
    // HMC: 1=P, 2=R, 3=N, 5=D.  [VERIFY] against vehicle-specific transmission.
    switch (ad01.target_gear) {
      case 0x01:  // Park
        control.gear = 0;
        control.hand_brake = true;
        control.reverse = false;
        break;
      case 0x02:  // Reverse
        control.gear = -1;
        control.hand_brake = false;
        control.reverse = true;
        break;
      case 0x03:  // Neutral
        control.gear = 0;
        control.hand_brake = false;
        control.reverse = false;
        break;
      case 0x05:  // Drive
        control.gear = 1;
        control.hand_brake = false;
        control.reverse = false;
        break;
      default:
        control.gear = 0;
        control.hand_brake = true;
        control.reverse = false;
        break;
    }

    if (ad01.stop_hold_req != 0u) {
      control.hand_brake = true;
    }

    return control;
  }

  bool HmcCommandSubscriber::HasFreshCommand() const {
    return std::chrono::steady_clock::now() - _last_command_time < kCommandTimeout;
  }

  void HmcCommandSubscriber::ProcessMessages(ActorCallback callback) {
    auto control = this->GetMessage();
    callback(this->GetActor(), control);
  }

}  // namespace ros2
}  // namespace carla
