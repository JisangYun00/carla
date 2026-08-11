// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>

#include <compiler/disable-ue4-macros.h>
#include "carla/ros2/ROS2.h"
#include <compiler/enable-ue4-macros.h>

/// visitor class
class ActorROS2Handler
{
    public:
        ActorROS2Handler() = delete;
        ActorROS2Handler(AActor *Actor, std::string RosName) : _Actor(Actor), _RosName(RosName) {};

    void operator()(carla::ros2::VehicleControl &Source);
    void operator()(carla::ros2::AckermannControl &Source);
    void PublishHmcFeedback();
    void PublishHmcVehicleStatus();
    void PublishHmcVehicleConfig();

    private:
        AActor *_Actor {nullptr};
        std::string _RosName;

        // Last commanded values remembered for FB-01 target echo.
        float _last_target_steer_ratio {0.0f};
        bool _last_stop_hold {false};
        uint8_t _last_target_gear {0x03}; // Neutral
};
