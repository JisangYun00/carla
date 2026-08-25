// Copyright (c) 2026 Hanyang University.
// Developed by Automotive Intelligence Lab.
// SPDX-License-Identifier: MIT
//
/**
 * Module:      CarlaEgoVehiclePhysicalStatusPublisher.h
 * Description: carla_msgs::msg::CarlaEgoVehiclePhysicalStatus publisher.
 *              Publishes the atomic vehicle physical status snapshot on
 *              rt/carla/ego_vehicle_physical_status.
 *
 * Authors:
 *   Jisang Yun (jisangyun@hanyang.ac.kr)
 *
 * Revision History:
 *   2026-08-25: Jisang Yun - Created.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "carla/ros2/publishers/BasePublisher.h"
#include "carla/ros2/publishers/PublisherImpl.h"

#include "carla/ros2/types/msg/CarlaEgoVehiclePhysicalStatus.h"

namespace carla {
namespace ros2 {

  // CARLA ego-vehicle physical status provider publisher.
  // Publishes a single atomic message containing vehicle geometry, kinematics,
  // chassis flags and a per-signal validity mask.
  class CarlaEgoVehiclePhysicalStatusPublisher : public BasePublisher {
    public:
      struct Traits {
        using msg_type = msg::CarlaEgoVehiclePhysicalStatus;
      };

      CarlaEgoVehiclePhysicalStatusPublisher() :
        BasePublisher("rt/carla/ego_vehicle_physical_status"),
        _impl(std::make_shared<PublisherImpl<Traits>>()) {
          if (!_impl->Init("rt/carla/ego_vehicle_physical_status")) {
            log_warning("CarlaEgoVehiclePhysicalStatusPublisher: Init failed");
          }
      }

      bool Publish() override {
        return _impl->Publish();
      }

      bool Write(
          int32_t stamp_sec,
          uint32_t stamp_nanosec,
          const std::string& frame_id,
          bool brake_status,
          bool abs_status,
          bool tcs_status,
          bool esc_status,
          uint8_t current_gear,
          float yaw_rate_radps,
          float lateral_acceleration_mps2,
          float longitudinal_acceleration_mps2,
          float steering_wheel_angle_deg,
          float vehicle_width_m,
          float vehicle_length_m,
          float vehicle_speed_kmh,
          bool ignition_status,
          uint64_t valid_signals);

#ifdef LIBCARLA_WITH_GTEST
      const msg::CarlaEgoVehiclePhysicalStatus* GetMessageForTesting() const {
        return _impl->GetMessage();
      }
#endif

    private:
      std::shared_ptr<PublisherImpl<Traits>> _impl;
  };

}  // namespace ros2
}  // namespace carla
