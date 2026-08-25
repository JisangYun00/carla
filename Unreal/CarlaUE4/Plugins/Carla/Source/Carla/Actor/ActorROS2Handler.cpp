// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ActorROS2Handler.h"

#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"
#include "Carla/Vehicle/VehicleAckermannControl.h"

#include <cmath>
#include <cstdlib>

namespace {

// Provider-owned steering-wheel to road-wheel ratio calibration.
// Returns a positive value when the environment supplies a valid calibration;
// otherwise returns 0.0f, which causes the SWA validity bit to stay cleared.
float GetSteeringRatioCalibration() {
  static const float ratio = []() -> float {
    const char* value = std::getenv("CARLA_HMC_STEERING_RATIO");
    if (!value) return 0.0f;
    char* end = nullptr;
    const float parsed = std::strtof(value, &end);
    if (end != value && *end == '\0' &&
        parsed > 0.0f && std::isfinite(parsed)) {
      return parsed;
    }
    return 0.0f;
  }();
  return ratio;
}

// Convert normalized CARLA steer command [-1, 1] to physical road-wheel
// angle using the vehicle's maximum road-wheel angle. Result is degrees.
float NormalizedSteerToRoadWheelDeg(const ACarlaWheeledVehicle* Vehicle, float normalized_steer) {
  if (!Vehicle) return 0.0f;
  const float MaxRoadWheelDeg = Vehicle->GetMaximumSteerAngle();
  if (!FMath::IsFinite(MaxRoadWheelDeg)) return 0.0f;
  return normalized_steer * MaxRoadWheelDeg;
}

} // namespace

void ActorROS2Handler::PublishEgoVehiclePhysicalStatus()
{
  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (!ROS2 || !ROS2->IsEnabled()) return;

  const FVehicleControl &Control = Vehicle->GetVehicleControl();

  // ---------------------------------------------------------------------------
  // Chassis flags
  // ---------------------------------------------------------------------------
  const bool BrakeStatus = (Control.Brake > 0.01f);

  // ---------------------------------------------------------------------------
  // Current gear: CARLA exposes the actual transmission gear.
  //   gear < 0  -> Reverse
  //   gear == 0 -> Neutral
  //   gear > 0  -> Drive
  // Park is not distinguishable, so it is never published.
  // ---------------------------------------------------------------------------
  const int32 Gear = Vehicle->GetVehicleCurrentGear();
  uint8_t CurrentGear = 5u;  // Drive default
  if (Gear < 0) CurrentGear = 2u;       // Reverse
  else if (Gear == 0) CurrentGear = 3u; // Neutral
  else CurrentGear = 5u;                // Drive

  // ---------------------------------------------------------------------------
  // Geometry (UE4 bounding box extent is in centimeters)
  // ---------------------------------------------------------------------------
  const FVector Extent = Vehicle->GetVehicleBoundingBoxExtent();
  float WidthM = Extent.Y * 2.0f * 0.01f;
  float LengthM = Extent.X * 2.0f * 0.01f;
  if (!FMath::IsFinite(WidthM) || WidthM <= 0.0f) {
    WidthM = 0.0f;
  }
  if (!FMath::IsFinite(LengthM) || LengthM <= 0.0f) {
    LengthM = 0.0f;
  }

  // ---------------------------------------------------------------------------
  // Actor-frame physics from the root primitive component.
  // UE4 actor frame: X forward, Y right, Z up (left-handed).
  // ROS vehicle frame: X forward, Y left, Z up.
  // Mapping matches CarlaIMUPublisher:  ros_y = -actor_y,  ros_z = -actor_z.
  // ---------------------------------------------------------------------------
  float YawRateRadps = 0.0f;
  float LateralAccelerationMps2 = 0.0f;
  float LongitudinalAccelerationMps2 = 0.0f;
  float SteeringWheelAngleDeg = 0.0f;
  float SpeedKmh = 0.0f;
  bool AbsStatus = false;
  bool TcsStatus = false;
  bool EscStatus = false;
  bool IgnitionStatus = false;
  uint64_t ValidSignals = 0u;

  // Gear is always considered valid because the mapping is well-defined.
  ValidSignals |= (1u << 4);  // current_gear

  UPrimitiveComponent* RootComponent = Cast<UPrimitiveComponent>(_Actor->GetRootComponent());
  if (RootComponent) {
    const FQuat ActorRotation = RootComponent->GetComponentTransform().GetRotation();

    // Speed magnitude from global linear velocity (cm/s -> km/h).
    const FVector GlobalVelocityCmps = RootComponent->GetPhysicsLinearVelocity();
    const FVector GlobalVelocityMps = GlobalVelocityCmps * 0.01f;
    if (GlobalVelocityCmps.IsZero()) {
      SpeedKmh = 0.0f;
    } else {
      SpeedKmh = GlobalVelocityCmps.Size() * 0.036f;
    }
    if (FMath::IsFinite(SpeedKmh)) {
      ValidSignals |= (1u << 11);  // vehicle_speed_kmh
    }

    // Yaw rate: actor-frame angular velocity Z mapped to ROS vehicle frame.
    const FVector GlobalAngularVelocity = RootComponent->GetPhysicsAngularVelocityInRadians();
    const FVector ActorAngularVelocity = ActorRotation.UnrotateVector(GlobalAngularVelocity);
    if (FMath::IsFinite(ActorAngularVelocity.Z)) {
      YawRateRadps = -ActorAngularVelocity.Z;
      ValidSignals |= (1u << 5);  // yaw_rate_radps
    }

    // Lateral / longitudinal acceleration: derivative of global velocity,
    // then rotated into the current actor frame.
    int32_t CurrentSec = 0;
    uint32_t CurrentNsec = 0u;
    ROS2->GetTimestamp(CurrentSec, CurrentNsec);
    const double CurrentTimeSec =
        static_cast<double>(CurrentSec) +
        static_cast<double>(CurrentNsec) * 1e-9;

    if (_has_last_global_velocity) {
      const double Dt = CurrentTimeSec - _last_global_velocity_time_sec;
      if (std::isfinite(Dt) && Dt > 1e-6) {
        const FVector GlobalAccMps2 =
            (GlobalVelocityMps - _last_global_velocity_mps) / Dt;
        if (FMath::IsFinite(GlobalAccMps2.X) &&
            FMath::IsFinite(GlobalAccMps2.Y)) {
          const FVector ActorAccMps2 = ActorRotation.UnrotateVector(GlobalAccMps2);
          LongitudinalAccelerationMps2 = ActorAccMps2.X;
          LateralAccelerationMps2 = -ActorAccMps2.Y;
          ValidSignals |= (1u << 6);   // lateral_acceleration_mps2
          ValidSignals |= (1u << 7);   // longitudinal_acceleration_mps2
        }
      }
    }

    _last_global_velocity_mps = GlobalVelocityMps;
    _last_global_velocity_time_sec = CurrentTimeSec;
    _has_last_global_velocity = true;
  }

  // ---------------------------------------------------------------------------
  // Geometry validity: positive finite dimensions only.
  // ---------------------------------------------------------------------------
  if (FMath::IsFinite(WidthM) && WidthM > 0.0f) {
    ValidSignals |= (1u << 9);   // vehicle_width_m
  }
  if (FMath::IsFinite(LengthM) && LengthM > 0.0f) {
    ValidSignals |= (1u << 10);  // vehicle_length_m
  }

  // ---------------------------------------------------------------------------
  // Brake status validity: finite threshold result.
  // ---------------------------------------------------------------------------
  if (FMath::IsFinite(Control.Brake)) {
    ValidSignals |= (1u << 0);  // brake_status
  }

  // ---------------------------------------------------------------------------
  // Steering wheel angle from actual wheel steer angle and provider calibration.
  // Uses the average of the front wheels when both are available.
  // ---------------------------------------------------------------------------
  const float SteeringRatio = GetSteeringRatioCalibration();
  if (SteeringRatio > 0.0f && FMath::IsFinite(SteeringRatio)) {
    const float FlSteer = Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FrontLeft);
    const float FrSteer = Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FrontRight);
    float RoadWheelAngleDeg = 0.0f;
    bool HaveValidWheelAngle = false;
    if (FMath::IsFinite(FlSteer) && FMath::IsFinite(FrSteer)) {
      RoadWheelAngleDeg = (FlSteer + FrSteer) * 0.5f;
      HaveValidWheelAngle = true;
    } else if (FMath::IsFinite(FlSteer)) {
      RoadWheelAngleDeg = FlSteer;
      HaveValidWheelAngle = true;
    }
    if (HaveValidWheelAngle) {
      const float SwaDeg = RoadWheelAngleDeg * SteeringRatio;
      if (FMath::IsFinite(SwaDeg)) {
        SteeringWheelAngleDeg = SwaDeg;
        ValidSignals |= (1u << 8);  // steering_wheel_angle_deg
      }
    }
  }

  // ---------------------------------------------------------------------------
  // ABS/TCS/ESC/ignition are not exposed by CARLA: publish false with bits clear.
  // ---------------------------------------------------------------------------

  ROS2->PublishEgoVehiclePhysicalStatus(
      _Actor, "vehicle",
      BrakeStatus,
      AbsStatus,
      TcsStatus,
      EscStatus,
      CurrentGear,
      YawRateRadps,
      LateralAccelerationMps2,
      LongitudinalAccelerationMps2,
      SteeringWheelAngleDeg,
      WidthM,
      LengthM,
      SpeedKmh,
      IgnitionStatus,
      ValidSignals);
}

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

  // FB-01 actual speed: forward speed in cm/s -> km/h. This is the signed
  // forward component, matching the legacy HMC feedback convention.
  const float SpeedKmh = Vehicle->GetVehicleForwardSpeed() * 0.036f;

  // Steering calibration governs whether SWA feedback is meaningful.
  const float SteeringRatio = GetSteeringRatioCalibration();
  const bool bCalibrationValid =
      SteeringRatio > 0.0f && FMath::IsFinite(SteeringRatio);

  float ActualSwaDeg = 0.0f;
  float TargetSwaEchoDeg = 0.0f;
  bool bActualSwaValid = false;
  bool bTargetSwaEchoValid = false;
  if (bCalibrationValid) {
    const float FlSteer = Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FrontLeft);
    const float FrSteer = Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FrontRight);
    float ActualRoadWheelDeg = 0.0f;
    bool HaveActual = false;
    if (FMath::IsFinite(FlSteer) && FMath::IsFinite(FrSteer)) {
      ActualRoadWheelDeg = (FlSteer + FrSteer) * 0.5f;
      HaveActual = true;
    } else if (FMath::IsFinite(FlSteer)) {
      ActualRoadWheelDeg = FlSteer;
      HaveActual = true;
    }
    if (HaveActual) {
      const float SwaDeg = ActualRoadWheelDeg * SteeringRatio;
      if (FMath::IsFinite(SwaDeg)) {
        ActualSwaDeg = SwaDeg;
        bActualSwaValid = true;
      }
    }

    const float TargetRoadWheelDeg =
        NormalizedSteerToRoadWheelDeg(Vehicle, _last_target_steer_ratio);
    const float EchoDeg = TargetRoadWheelDeg * SteeringRatio;
    if (FMath::IsFinite(EchoDeg)) {
      TargetSwaEchoDeg = EchoDeg;
      bTargetSwaEchoValid = true;
    }
  }

  const bool bSpeedFinite = FMath::IsFinite(SpeedKmh);
  const bool bThrottleBrakeFinite =
      FMath::IsFinite(Control.Throttle) && FMath::IsFinite(Control.Brake);
  const bool bActuatorFault =
      !bSpeedFinite ||
      !bThrottleBrakeFinite ||
      !bActualSwaValid ||
      !bTargetSwaEchoValid ||
      !bCalibrationValid;
  const uint8_t lng_ready = bCalibrationValid ? 1u : 0u;
  const uint8_t lat_ready = (bCalibrationValid && bActualSwaValid && bTargetSwaEchoValid) ? 1u : 0u;
  const uint8_t gear_ready = 1u;  // gear mapping is always well-defined

  (void)Control.Steer;  // No longer used for SWA; avoid unused warning.

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
      bActuatorFault,
      lng_ready,
      lat_ready,
      gear_ready);
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
