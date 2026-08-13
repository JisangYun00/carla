// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "carla/ros2/publishers/BasePublisher.h"
#include "carla/ros2/publishers/PublisherImpl.h"

#include "carla/ros2/types/msg/HmcFB01.h"

namespace carla {
namespace ros2 {

  // HMC FB-01 motion control status feedback publisher.
  // Publishes vehicle state on fixed topic rt/hmc/fb/fb01.
  class HmcFeedbackPublisher : public BasePublisher {
    public:
      struct FB01MsgTraits {
        using msg_type = msg::HmcFB01;
      };

      HmcFeedbackPublisher() :
        BasePublisher("rt/hmc/fb"),
        _impl(std::make_shared<PublisherImpl<FB01MsgTraits>>()) {
          if (!_impl->Init("rt/hmc/fb/fb01")) {
            log_warning("HmcFeedbackPublisher: Init failed for topic: rt/hmc/fb/fb01");
          }
      }

      bool Publish() override {
        return _impl->Publish();
      }

      // Fill FB-01 fields from CARLA actor state.
      bool Write(
          uint8_t alive_counter,
          float aps_pct,
          float bps_pct,
          float actual_speed_kmh,
          float target_speed_echo_kmh,
          float actual_swa_deg,
          float target_swa_echo_deg,
          uint8_t lng_op_mode,
          uint8_t lat_op_mode,
          bool actuator_fault,
          uint8_t lng_ctrl_ready,
          uint8_t lat_ctrl_ready,
          uint8_t gear_sel_ready);

    private:
      std::shared_ptr<PublisherImpl<FB01MsgTraits>> _impl;
  };

}  // namespace ros2
}  // namespace carla
