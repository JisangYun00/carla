// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <functional>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4583)
#pragma warning(disable:4582)
#include <boost/variant2/variant.hpp>
#pragma warning(pop)
#else
#include <boost/variant2/variant.hpp>
#endif

namespace carla {
namespace ros2 {

  struct VehicleControl
  {
    float   throttle;
    float   steer;
    float   brake;
    bool    hand_brake;
    bool    reverse;
    int32_t gear;
    bool    manual_gear_shift;
    // HMC commands carry physical steering-wheel degrees; native CARLA
    // commands keep normalized steer in [-1, 1].
    bool    steer_is_steering_wheel_angle {false};
    // HMC AD-01 carries a speed target, unlike the native CARLA pedal command.
    bool    hmc_command {false};
    bool    speed_control {false};
    bool    ad01_fresh {false};
    bool    emergency_brake {false};
    float   target_speed_kmh {0.0f};
  };

  struct AckermannControl
  {
    float steer;
    float steer_speed;
    float speed;
    float acceleration;
    float jerk;
  };

  using ROS2CallbackData = boost::variant2::variant<
    VehicleControl,
    AckermannControl
  >;

  using ActorCallback = std::function<void(void *actor, ROS2CallbackData data)>;

} // namespace ros2
} // namespace carla
