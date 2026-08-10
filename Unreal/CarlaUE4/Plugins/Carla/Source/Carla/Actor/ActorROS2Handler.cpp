// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ActorROS2Handler.h"

#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"
#include "Carla/Vehicle/VehicleAckermannControl.h"

void ActorROS2Handler::operator()(carla::ros2::VehicleControl &Source)
{
  if (!_Actor) return;

  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  // setup control values
  FVehicleControl NewControl;
  NewControl.Throttle = Source.throttle;
  NewControl.Steer = Source.steer;
  NewControl.Brake = Source.brake;
  NewControl.bHandBrake = Source.hand_brake;
  NewControl.bReverse = Source.reverse;
  NewControl.bManualGearShift = Source.manual_gear_shift;
  NewControl.Gear = Source.gear;

  Vehicle->ApplyVehicleControl(NewControl, EVehicleInputPriority::User);

  // Publish HMC FB-01 feedback using the resulting vehicle state.
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2 && ROS2->IsEnabled())
  {
    const float SpeedCmS = Vehicle->GetVehicleForwardSpeed();
    const float SpeedKmh = SpeedCmS * 0.036f;  // cm/s -> km/h
    const float SteerDeg = NewControl.Steer * 540.0f;  // [VERIFY] max steer angle
    const int32 CurrentGear = Vehicle->GetVehicleCurrentGear();
    // Simple actuator-fault heuristic: non-finite command or out-of-range steer.
    const bool bActuatorFault =
        !FMath::IsFinite(NewControl.Throttle) ||
        !FMath::IsFinite(NewControl.Steer) ||
        FMath::Abs(NewControl.Steer) > 1.0f;
    ROS2->PublishHmcFeedback(
        _Actor,
        NewControl.Throttle * 100.0f,    // aps_fdb_pct physical 0..100
        NewControl.Brake * 100.0f,       // bps_fdb_pct physical 0..100
        SpeedKmh,
        0.0f,                            // target_speed_echo_kmh [TODO]
        SteerDeg,
        SteerDeg,                        // target_swa_echo_deg [TODO]
        0x01,                            // lng_op_mode [VERIFY]
        0x01,                            // lat_op_mode [VERIFY]
        bActuatorFault);
  }
}

void ActorROS2Handler::operator()(carla::ros2::AckermannControl &Source)
{
  if (!_Actor) return;

  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  // setup control values
  FVehicleAckermannControl NewControl;
  NewControl.Steer = Source.steer;
  NewControl.SteerSpeed = Source.steer_speed;
  NewControl.Speed = Source.speed;
  NewControl.Acceleration = Source.acceleration;
  NewControl.Jerk = Source.jerk;

  Vehicle->ApplyVehicleAckermannControl(NewControl, EVehicleInputPriority::User);
}
