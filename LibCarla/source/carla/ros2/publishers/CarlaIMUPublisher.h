// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "carla/geom/Vector3D.h"

#include "carla/ros2/publishers/BasePublisher.h"
#include "carla/ros2/publishers/PublisherImpl.h"

#include "carla/ros2/types/msg/Imu.h"

namespace carla {
namespace ros2 {

  // CARLA compass is clockwise from North; ROS ENU yaw is counter-clockwise
  // from East. Cardinal checks: East π/2→0, North 0→π/2.
  constexpr float CompassToRosYaw(float compass) {
    return 1.5707963267948966f - compass;
  }

  class CarlaIMUPublisher : public BasePublisher {
    public:
      struct ImuMsgTraits {
        using msg_type = msg::Imu;
      };

      CarlaIMUPublisher(std::string base_topic_name, std::string frame_id) :
        BasePublisher(base_topic_name, frame_id),
        _impl(std::make_shared<PublisherImpl<ImuMsgTraits>>()) {
          if (!_impl->Init(this->GetBaseTopicName())) {
            log_warning("CarlaIMUPublisher: Init failed for topic: ", this->GetBaseTopicName());
          }
      }

      bool Publish() {
        return _impl->Publish();
      }

      bool Write(int32_t seconds, uint32_t nanoseconds, geom::Vector3D accelerometer, geom::Vector3D gyroscope, float compass);

    private:
      std::shared_ptr<PublisherImpl<ImuMsgTraits>> _impl;
  };

}  // namespace ros2
}  // namespace carla
