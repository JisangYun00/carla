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

  // Remember the commanded values for FB-01 target echo.
  _last_target_steer_ratio = Source.steer;
  _last_stop_hold = Source.hand_brake;
  _last_target_gear = static_cast<uint8_t>(
      Source.reverse ? 0x02 :
      Source.hand_brake ? 0x01 :
      Source.gear == 0 ? 0x03 : 0x05);
}

void ActorROS2Handler::PublishHmcFeedback()
{
  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (!ROS2 || !ROS2->IsEnabled()) return;

  const FVehicleControl &Control = Vehicle->GetVehicleControl();
  const float SpeedKmh = Vehicle->GetVehicleForwardSpeed() * 0.036f;
  const float ActualSwaDeg = Control.Steer * 540.0f;  // [VERIFY] max steer angle
  const float TargetSwaEchoDeg = _last_target_steer_ratio * 540.0f;
  const bool bActuatorFault =
      !FMath::IsFinite(Control.Throttle) ||
      !FMath::IsFinite(Control.Steer) ||
      !FMath::IsFinite(Control.Brake);
  ROS2->PublishHmcFeedback(
      _Actor,
      Control.Throttle * 100.0f,
      Control.Brake * 100.0f,
      SpeedKmh,
      0.0f,                    // target_speed_echo_kmh [TODO]
      ActualSwaDeg,
      TargetSwaEchoDeg,
      0x01,                    // lng_op_mode [VERIFY]
      0x01,                    // lat_op_mode [VERIFY]
      bActuatorFault);
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
