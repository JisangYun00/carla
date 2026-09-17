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

float GetPositiveHmcEnv(const char* name, float fallback) {
  const char* value = std::getenv(name);
  if (!value) return fallback;
  char* end = nullptr;
  const float parsed = std::strtof(value, &end);
  return end != value && *end == '\0' && parsed > 0.0f && std::isfinite(parsed)
      ? parsed : fallback;
}

// Speed-loop calibration: percentage-points per km/h, percentage-points per
// km/h-second, and maximum accelerator percentage respectively.
const float kSpeedKp = GetPositiveHmcEnv("CARLA_HMC_SPEED_KP", 2.0f);
const float kSpeedKi = GetPositiveHmcEnv("CARLA_HMC_SPEED_KI", 0.5f);
const float kSpeedApsLimit = GetPositiveHmcEnv("CARLA_HMC_SPEED_APS_LIMIT", 40.0f);

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

// Effective SWA full-scale calibration. CARLA's configured wheel steer limit
// is an input limit; the measured front-wheel response is vehicle-specific.
float GetMaxSwaCalibration() {
  static const float max_swa = []() -> float {
    const char* value = std::getenv("CARLA_HMC_MAX_SWA_DEG");
    if (!value) return 0.0f;
    char* end = nullptr;
    const float parsed = std::strtof(value, &end);
    if (end != value && *end == '\0' &&
        parsed > 0.0f && std::isfinite(parsed)) {
      return parsed;
    }
    return 0.0f;
  }();
  return max_swa;
}

// Steering-column angle equivalent to the measured front-wheel angles. CARLA
// has no steering-column sensor, so Safety compares its AD-01 target against
// this measurement rather than against an echo of its own command.
float MeasuredSwaDeg(ACarlaWheeledVehicle* Vehicle, float SteeringRatio) {
  const float Fl = Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FL_Wheel);
  const float Fr = Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FR_Wheel);
  if (!FMath::IsFinite(Fl) || !FMath::IsFinite(Fr)) return NAN;
  return (Fl + Fr) * 0.5f * SteeringRatio;
}

} // namespace

void ActorROS2Handler::PublishVehiclePhysicalStatus()
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
  float WheelAngularVelocityFlRadps = 0.0f;
  float WheelAngularVelocityFrRadps = 0.0f;
  float WheelAngularVelocityRlRadps = 0.0f;
  float WheelAngularVelocityRrRadps = 0.0f;
  bool AbsStatus = false;
  bool TcsStatus = false;
  bool EscStatus = false;
  bool IgnitionStatus = false;
  uint64_t ValidFields = 0u;

  // Gear is always considered valid because the mapping is well-defined.
  ValidFields |= (1u << 4);  // current_gear

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
      ValidFields |= (1u << 11);  // vehicle_speed_kmh
    }

    // Yaw rate: actor-frame angular velocity Z mapped to ROS vehicle frame.
    const FVector GlobalAngularVelocity = RootComponent->GetPhysicsAngularVelocityInRadians();
    const FVector ActorAngularVelocity = ActorRotation.UnrotateVector(GlobalAngularVelocity);
    if (FMath::IsFinite(ActorAngularVelocity.Z)) {
      YawRateRadps = -ActorAngularVelocity.Z;
      ValidFields |= (1u << 5);  // yaw_rate_radps
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
          ValidFields |= (1u << 6);   // lateral_acceleration_mps2
          ValidFields |= (1u << 7);   // longitudinal_acceleration_mps2
        }
      }
    }

    _last_global_velocity_mps = GlobalVelocityMps;
    _last_global_velocity_time_sec = CurrentTimeSec;
    _has_last_global_velocity = true;
  }

  // Do not call GetVehicleTelemetryData here. It dereferences the PhysX
  // PVehicle before it exists during actor initialization and can crash CARLA.
  // Wheel angular velocity fields remain invalid (their values stay zero) until
  // CARLA exposes a lifecycle-safe public API for this data.

  // ---------------------------------------------------------------------------
  // Geometry validity: positive finite dimensions only.
  // ---------------------------------------------------------------------------
  if (FMath::IsFinite(WidthM) && WidthM > 0.0f) {
    ValidFields |= (1u << 9);   // vehicle_width_m
  }
  if (FMath::IsFinite(LengthM) && LengthM > 0.0f) {
    ValidFields |= (1u << 10);  // vehicle_length_m
  }

  // ---------------------------------------------------------------------------
  // Brake status validity: finite threshold result.
  // ---------------------------------------------------------------------------
  if (FMath::IsFinite(Control.Brake)) {
    ValidFields |= (1u << 0);  // brake_status
  }

  // ---------------------------------------------------------------------------
  // Steering-wheel angle derived from the measured front wheels.
  // ---------------------------------------------------------------------------
  const float PhysicalSteeringRatio = GetSteeringRatioCalibration();
  if (PhysicalSteeringRatio > 0.0f && FMath::IsFinite(PhysicalSteeringRatio)) {
    const float MeasuredDeg = MeasuredSwaDeg(Vehicle, PhysicalSteeringRatio);
    if (FMath::IsFinite(MeasuredDeg)) {
      SteeringWheelAngleDeg = MeasuredDeg;
      ValidFields |= (1u << 8);  // steering_wheel_angle_deg
    }
  }

  // ---------------------------------------------------------------------------
  // ABS/TCS/ESC/ignition are not exposed by CARLA: publish false with bits clear.
  // ---------------------------------------------------------------------------

  ROS2->PublishVehiclePhysicalStatus(
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
      WheelAngularVelocityFlRadps,
      WheelAngularVelocityFrRadps,
      WheelAngularVelocityRlRadps,
      WheelAngularVelocityRrRadps,
      IgnitionStatus,
      ValidFields);
}

void ActorROS2Handler::operator()(carla::ros2::VehicleControl &Source)
{
  if (!_Actor) return;

  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  float NormalizedSteer = Source.steer;
  if (Source.steer_is_steering_wheel_angle) {
    const float SteeringRatio = GetSteeringRatioCalibration();
    const float MaxRoadWheelDeg = Vehicle->GetMaximumSteerAngle();
    const float CalibratedMaxSwaDeg = GetMaxSwaCalibration();
    const float MaxSwaDeg =
        CalibratedMaxSwaDeg > 0.0f
        ? CalibratedMaxSwaDeg
        : SteeringRatio * MaxRoadWheelDeg;
    NormalizedSteer =
        FMath::IsFinite(MaxSwaDeg) && MaxSwaDeg > 0.0f
            ? FMath::Clamp(Source.steer / MaxSwaDeg, -1.0f, 1.0f)
            : 0.0f;
  }

  float Throttle = Source.throttle;
  float Brake = Source.brake;
  if (Source.hmc_command) {
    int32_t Sec = 0;
    uint32_t Nsec = 0u;
    carla::ros2::ROS2::GetInstance()->GetTimestamp(Sec, Nsec);
    const double Now = static_cast<double>(Sec) + static_cast<double>(Nsec) * 1e-9;
    const float Dt = _last_actuator_time_sec >= 0.0 && Now > _last_actuator_time_sec
        ? FMath::Clamp(static_cast<float>(Now - _last_actuator_time_sec), 0.0f, 0.1f)
        : 0.01f;
    _last_actuator_time_sec = Now;

    const bool TargetValid = Source.target_speed_kmh >= 0.0f &&
        Source.target_speed_kmh <= 30.0f && FMath::IsFinite(Source.target_speed_kmh);
    if (Source.emergency_brake || (Source.speed_control && !TargetValid)) {
      _speed_integral_kmh_sec = 0.0f;
      Throttle = 0.0f;
      Brake = 1.0f;
    } else if (Source.speed_control) {
      const float SpeedErrorKmh = Source.target_speed_kmh -
          FMath::Abs(Vehicle->GetVehicleForwardSpeed()) * 0.036f;
      if (Source.ad01_fresh && _has_target_speed_echo &&
          FMath::Abs(Source.target_speed_kmh - _last_target_speed_kmh) > 2.0f) {
        _speed_integral_kmh_sec = 0.0f;
      }
      const float ProportionalPct = kSpeedKp * SpeedErrorKmh;
      if (FMath::Abs(SpeedErrorKmh) > 0.3f) {
        const float CandidateIntegral = _speed_integral_kmh_sec + SpeedErrorKmh * Dt;
        const float UnsaturatedPct = ProportionalPct + kSpeedKi * CandidateIntegral;
        if (!((UnsaturatedPct >= kSpeedApsLimit && SpeedErrorKmh > 0.0f) ||
              (UnsaturatedPct <= 0.0f && SpeedErrorKmh < 0.0f))) {
          _speed_integral_kmh_sec = CandidateIntegral;
        }
      }
      const float PedalPct = ProportionalPct + kSpeedKi * _speed_integral_kmh_sec;
      Throttle = FMath::Clamp(PedalPct / 100.0f, 0.0f, kSpeedApsLimit / 100.0f);
      Brake = FMath::Clamp(-PedalPct / 100.0f, 0.0f, 1.0f);
      if (Brake > 0.0f) {
        _speed_integral_kmh_sec = 0.0f;
      }
    } else {
      _speed_integral_kmh_sec = 0.0f;
    }
    if (Source.ad01_fresh && TargetValid) {
      _last_target_speed_kmh = Source.target_speed_kmh;
      _has_target_speed_echo = true;
    }
  }

  FVehicleControl NewControl;
  NewControl.Throttle = Throttle;
  NewControl.Steer = NormalizedSteer;
  NewControl.Brake = Brake;
  NewControl.bHandBrake = Source.hand_brake;
  NewControl.bReverse = Source.reverse;
  NewControl.bManualGearShift = Source.manual_gear_shift;
  NewControl.Gear = Source.gear;

  // HMC DDS commands are external vehicle commands. Give them the same
  // priority as CARLA RPC client control so they reach the movement component
  // rather than being relaxed as local user input.
  Vehicle->ApplyVehicleControl(
      NewControl,
      Source.hmc_command ? EVehicleInputPriority::Client
                         : EVehicleInputPriority::User);

  // Keep the unmodified HMC target as the FB-01 command echo. The feedback
  // field below is the delayed virtual actuator position, not wheel angle.
  if (Source.hmc_command && Source.steer_is_steering_wheel_angle) {
    _last_target_swa_deg = Source.steer;
    _has_target_swa_echo = true;
  }
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

  // FB-01 reports speed magnitude; direction is represented by actual gear.
  const float SpeedKmh = FMath::Abs(Vehicle->GetVehicleForwardSpeed()) * 0.036f;

  const float SteeringRatio = GetSteeringRatioCalibration();
  const bool bCalibrationValid =
      SteeringRatio > 0.0f && FMath::IsFinite(SteeringRatio);
  int32_t Sec = 0;
  uint32_t Nsec = 0u;
  ROS2->GetTimestamp(Sec, Nsec);
  const double Now = static_cast<double>(Sec) + static_cast<double>(Nsec) * 1e-9;

  const float ActualSwaDeg =
      bCalibrationValid ? MeasuredSwaDeg(Vehicle, SteeringRatio) : NAN;
  const float TargetSwaEchoDeg = _last_target_swa_deg;
  const bool bActualSwaValid = FMath::IsFinite(ActualSwaDeg);
  const bool bTargetSwaEchoValid =
      _has_target_swa_echo && FMath::IsFinite(TargetSwaEchoDeg);

  const bool bSpeedFinite = FMath::IsFinite(SpeedKmh);
  const bool bThrottleBrakeFinite =
      FMath::IsFinite(Control.Throttle) && FMath::IsFinite(Control.Brake);
  const bool bActuatorFault =
      !bSpeedFinite ||
      !bThrottleBrakeFinite ||
      !bActualSwaValid ||
      !bTargetSwaEchoValid ||
      !bCalibrationValid;
  const uint8_t lng_ready = (!bActuatorFault && bSpeedFinite && bThrottleBrakeFinite) ? 1u : 0u;
  const uint8_t lat_ready = (!bActuatorFault && bActualSwaValid && bTargetSwaEchoValid) ? 1u : 0u;
  const bool Stopped = bSpeedFinite && SpeedKmh <= 0.36f;
  if (Stopped) {
    if (_stationary_since_sec < 0.0) _stationary_since_sec = Now;
  } else {
    _stationary_since_sec = -1.0;
  }
  const bool StoppedForShift = Stopped && _stationary_since_sec >= 0.0 &&
      Now - _stationary_since_sec >= 0.5;
  const uint8_t gear_ready = (!bActuatorFault && StoppedForShift) ? 1u : 0u;
  const uint8_t stop_hold_ready = (!bActuatorFault && StoppedForShift) ? 1u : 0u;

  ROS2->PublishHmcFeedback(
      _Actor,
      Control.Throttle * 100.0f,
      Control.Brake * 100.0f,
      SpeedKmh,
      _has_target_speed_echo ? _last_target_speed_kmh : 0.0f,
      ActualSwaDeg,
      TargetSwaEchoDeg,
      0x01,                    // lng_op_mode [VERIFY]
      0x01,                    // lat_op_mode [VERIFY]
      bActuatorFault,
      lng_ready,
      lat_ready,
      gear_ready,
      stop_hold_ready);
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
