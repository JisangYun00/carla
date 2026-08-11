// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "carla/ros2/publishers/BasePublisher.h"
#include "carla/ros2/publishers/PublisherImpl.h"

#include "carla/ros2/types/msg/HmcVehicleConfig.h"

namespace carla {
namespace ros2 {

  // HMC vehicle configuration publisher.
  // Publishes vehicle geometry on fixed topic rt/ads/vehicle/configuration.
  class HmcVehicleConfigPublisher : public BasePublisher {
    public:
      struct Traits {
        using msg_type = msg::HmcVehicleConfig;
      };

      HmcVehicleConfigPublisher() :
        BasePublisher("rt/ads/vehicle"),
        _impl(std::make_shared<PublisherImpl<Traits>>()) {
          if (!_impl->Init("rt/ads/vehicle/configuration", PublisherQos{
              DurabilityKind::TransientLocal, ReliabilityKind::Reliable, 1u})) {
            log_warning("HmcVehicleConfigPublisher: Init failed");
          }
      }

      bool Publish() override {
        return _impl->Publish();
      }

      bool Write(
          int32_t stamp_sec,
          uint32_t stamp_nanosec,
          const std::string& frame_id,
          float vehicle_width_m,
          float vehicle_length_m,
          uint8_t ignition_default_on);

    private:
      std::shared_ptr<PublisherImpl<Traits>> _impl;
  };

}  // namespace ros2
}  // namespace carla