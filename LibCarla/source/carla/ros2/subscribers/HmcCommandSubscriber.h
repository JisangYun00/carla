// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <chrono>
#include <memory>

#include "BaseSubscriber.h"
#include "SubscriberImpl.h"

#include "carla/ros2/types/msg/HmcAD01.h"
#include "carla/ros2/types/msg/HmcAD02.h"

#include "carla/ros2/ROS2CallbackData.h"

namespace carla {
namespace ros2 {

  // Combined HMC AD-01 / AD-02 command subscriber.
  // Subscribes to fixed topic names /hmc/ad/ad01 and /hmc/ad/ad02 and converts
  // incoming HMC messages to the internal VehicleControl representation used by
  // the CARLA vehicle actor callback.
  class HmcCommandSubscriber : public BaseSubscriber {
    public:
      struct AD01MsgTraits {
        using msg_type = msg::HmcAD01;
      };

      struct AD02MsgTraits {
        using msg_type = msg::HmcAD02;
      };

      HmcCommandSubscriber(void* vehicle, std::string /*base_topic_name*/, std::string frame_id) :
        BaseSubscriber(vehicle, "rt/hmc/ad", frame_id),
        _ad01_impl(std::make_shared<SubscriberImpl<AD01MsgTraits>>()),
        _ad02_impl(std::make_shared<SubscriberImpl<AD02MsgTraits>>()),
        _last_command_time(std::chrono::steady_clock::now()) {
          if (!_ad01_impl->Init("rt/hmc/ad/ad01")) {
            log_warning("HmcCommandSubscriber: Init failed for topic: rt/hmc/ad/ad01");
          }
          if (!_ad02_impl->Init("rt/hmc/ad/ad02")) {
            log_warning("HmcCommandSubscriber: Init failed for topic: rt/hmc/ad/ad02");
          }
        }

      // Combine the latest AD-01 and AD-02 into one VehicleControl.
      // AD-01 provides normal lateral/longitudinal targets; AD-02 overrides in
      // emergency (emergency brake / steering).
      // If no fresh command arrives within the timeout, returns a fail-safe
      // control (full brake, zero throttle/steer).
      ROS2CallbackData GetMessage() override;

      // Returns true if a command has been received within the freshness window.
      bool HasFreshCommand() const;

      void ProcessMessages(ActorCallback callback) override;

    private:
      std::shared_ptr<SubscriberImpl<AD01MsgTraits>> _ad01_impl;
      std::shared_ptr<SubscriberImpl<AD02MsgTraits>> _ad02_impl;

      // Cached latest commands; used to merge AD-01 and AD-02 samples.
      msg::HmcAD01 _latest_ad01;
      msg::HmcAD02 _latest_ad02;

      std::chrono::steady_clock::time_point _last_command_time;
      static constexpr auto kCommandTimeout = std::chrono::milliseconds(30);
  };

}  // namespace ros2
}  // namespace carla
