// Copyright (c) 2026 Hanyang University
// Developed by Automotive Intelligence Lab
// SPDX-License-Identifier: MIT

#include "HmcCommandSubscriber.h"

#include <chrono>

#include "HmcCommandArbitration.h"
#include "carla/ros2/ROS2CallbackData.h"

namespace carla {
namespace ros2 {

  // Conversion helper from HMC raw SWA (0.1 degree) to CARLA input units.
  namespace {
    constexpr float kRawToPhysical = 0.1f;

    inline float raw_to_swa_deg(int16_t raw_deg) {
      return raw_deg * kRawToPhysical;
    }
  }

  ROS2CallbackData HmcCommandSubscriber::GetMessage() {
    auto ad01 = _latest_ad01;
    auto ad02 = _latest_ad02;
    const bool ad01_new = _ad01_impl->TakeMessage(ad01);
    const bool ad02_new = _ad02_impl->TakeMessage(ad02);

    const auto now = std::chrono::steady_clock::now();
    if (ad01_new) {
      _last_ad01_time = now;
      _ad01_received = true;
    }
    if (ad02_new) {
      _last_ad02_time = now;
    }

    // Cache latest values so AD-01/02 arriving at different times still merge.
    if (ad01_new) _latest_ad01 = ad01;
    if (ad02_new) _latest_ad02 = ad02;

    VehicleControl control{};
    const auto source = SelectHmcCommand(
        _ad01_received && now - _last_ad01_time < kCommandTimeout,
        ad02.emgc_brk_active != 0u,
        ad02.emgc_steer_active != 0u,
        now - _last_ad02_time,
        kCommandTimeout);

    // Command timeout: fail-safe brake if no fresh AD-01/AD-02.
    if (source == HmcCommandSource::Timeout) {
      control.brake = 1.0f;
      return control;
    }

    // AD-01 carries speed/steering targets. CARLA converts the speed target
    // to pedal inputs in ActorROS2Handler, the drive-controller boundary.
    control.hmc_command = true;
    control.ad01_fresh = ad01_new;
    control.speed_control = ad01.lng_ctrl_engage_req != 0u;
    control.target_speed_kmh = static_cast<float>(ad01.target_speed_kmh) * kRawToPhysical;
    control.steer_is_steering_wheel_angle = true;
    if (ad01.lat_ctrl_engage_req) {
      control.steer = raw_to_swa_deg(ad01.target_swa_deg);
    }

    // A fresh active AD-02 remains authoritative between its 10 ms samples.
    if (source == HmcCommandSource::AD02) {
      control.emergency_brake = ad02.emgc_brk_active != 0u;
      if (ad02.emgc_brk_active) {
        control.brake = 1.0f;
        control.throttle = 0.0f;
      }
      if (ad02.emgc_steer_active) {
        control.steer_is_steering_wheel_angle = true;
        control.steer = raw_to_swa_deg(ad02.emgc_steer_ang_tgt_deg);
      }
    }

    // Map HMC gear enum to CARLA VehicleControl fields.
    // HMC: 1=P, 2=R, 3=N, 5=D.  [VERIFY] against vehicle-specific transmission.
    switch (ad01.target_gear) {
      case 0x01:  // Park
        control.gear = 0;
        control.manual_gear_shift = true;
        control.hand_brake = true;
        control.reverse = false;
        break;
      case 0x02:  // Reverse
        control.gear = -1;
        control.manual_gear_shift = true;
        control.hand_brake = false;
        control.reverse = true;
        break;
      case 0x03:  // Neutral
        control.gear = 0;
        control.manual_gear_shift = true;
        control.hand_brake = false;
        control.reverse = false;
        break;
      case 0x05:  // Drive
        control.gear = 1;
        control.manual_gear_shift = true;
        control.hand_brake = false;
        control.reverse = false;
        break;
      default:
        control.gear = 0;
        control.manual_gear_shift = true;
        control.hand_brake = true;
        control.reverse = false;
        break;
    }

    if (ad01.stop_hold_req != 0u) {
      control.hand_brake = true;
    }

    return control;
  }

  void HmcCommandSubscriber::ProcessMessages(ActorCallback callback) {
    auto control = this->GetMessage();
    callback(this->GetActor(), control);
  }

}  // namespace ros2
}  // namespace carla
