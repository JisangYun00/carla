// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
/**
 * Module:      ActorROS2Handler.h
 * Description: ROS2 visitor/callback handler for a registered CARLA actor.
 *              Bridges AActor state to the CARLA ROS2 middleware publishers
 *              and forwards VehicleControl/AckermannControl commands back to
 *              the actor.
 *
 * Authors:
 *   Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB)
 *   Jisang Yun (jisangyun@hanyang.ac.kr)
 *
 * Revision History:
 *   2026-08-25: Jisang Yun - Replaced legacy HmcVehicleStatus/HmcVehicleConfig
 *               publishers with single CarlaEgoVehiclePhysicalStatusPublisher;
 *               updated FB-01 SWA to use real wheel angle + steering ratio.
 */

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
    void PublishEgoVehiclePhysicalStatus();

    private:
        AActor *_Actor {nullptr};
        std::string _RosName;

        // Last commanded values remembered for FB-01 target echo.
        float _last_target_steer_ratio {0.0f};
        bool _last_stop_hold {false};
        uint8_t _last_target_gear {0x03}; // Neutral

        // State for computing actor-frame acceleration from global velocity derivative.
        FVector _last_global_velocity_mps {FVector::ZeroVector};
        double _last_global_velocity_time_sec {0.0};
        bool _has_last_global_velocity {false};
};
