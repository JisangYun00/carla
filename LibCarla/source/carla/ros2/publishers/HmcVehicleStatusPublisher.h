// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "carla/ros2/publishers/BasePublisher.h"
#include "carla/ros2/publishers/PublisherImpl.h"

#include "carla/ros2/types/msg/HmcVehicleStatus.h"

namespace carla {
namespace ros2 {

  // HMC vehicle control-unit status publisher.
  // Publishes vehicle status on fixed topic rt/ads/vehicle/control_unit_state.
  class HmcVehicleStatusPublisher : public BasePublisher {
    public:
      struct Traits {
        using msg_type = msg::HmcVehicleStatus;
      };

      HmcVehicleStatusPublisher() :
        BasePublisher("rt/ads/vehicle"),
        _impl(std::make_shared<PublisherImpl<Traits>>()) {
          if (!_impl->Init("rt/ads/vehicle/control_unit_state")) {
            log_warning("HmcVehicleStatusPublisher: Init failed");
          }
      }

      bool Publish() override {
        return _impl->Publish();
      }

      bool Write(
          int32_t stamp_sec,
          uint32_t stamp_nanosec,
          const std::string& frame_id,
          uint8_t current_gear,
          uint8_t brake_status,
          float vehicle_speed_kmh,
          float actual_swa_deg,
          uint8_t ignition_on,
          uint64_t valid_flags);

    private:
      std::shared_ptr<PublisherImpl<Traits>> _impl;
  };

}  // namespace ros2
}  // namespace carla