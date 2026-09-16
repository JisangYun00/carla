// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/ROS2.h"

#include "carla/Logging.h"
#include "carla/ros2/middleware/MiddlewareFactory.h"
#include "carla/geom/GeoLocation.h"
#include "carla/geom/Vector3D.h"
#include "carla/sensor/data/DVSEvent.h"
#include "carla/sensor/data/LidarData.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/RadarData.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/s11n/ImageSerializer.h"
#include "carla/sensor/s11n/SensorHeaderSerializer.h"

#include "publishers/CarlaCameraPublisher.h"
#include "publishers/CarlaClockPublisher.h"
#include "publishers/CarlaCollisionPublisher.h"
#include "publishers/CarlaDepthCameraPublisher.h"
#include "publishers/CarlaDVSPublisher.h"
#include "publishers/CarlaGNSSPublisher.h"
#include "publishers/CarlaIMUPublisher.h"
#include "publishers/CarlaISCameraPublisher.h"
#include "publishers/CarlaLidarPublisher.h"
#include "publishers/CarlaMapPublisher.h"
#include "publishers/CarlaNormalsCameraPublisher.h"
#include "publishers/CarlaOpticalFlowCameraPublisher.h"
#include "publishers/CarlaRadarPublisher.h"
#include "publishers/CarlaRGBCameraPublisher.h"
#include "publishers/CarlaSemanticLidarPublisher.h"
#include "publishers/CarlaSSCameraPublisher.h"
#include "publishers/CarlaTransformPublisher.h"
#include "publishers/HmcFeedbackPublisher.h"
#include "publishers/CarlaVehiclePhysicalStatusPublisher.h"

#include "subscribers/AckermannControlSubscriber.h"
#include "subscribers/CarlaEgoVehicleControlSubscriber.h"
#include "subscribers/HmcCommandSubscriber.h"

#include <vector>
#include <cstdlib>
#include <cstring>

namespace carla {
namespace ros2 {

namespace {
  uint8_t GetEnvReady(const char* name) {
    const char* value = std::getenv(name);
    if (value) {
      if (std::strcmp(value, "0") == 0) return 0u;
      if (std::strcmp(value, "1") == 0) return 1u;
    }
    return 1u;
  }
}

// static fields
std::shared_ptr<ROS2> ROS2::_instance;

// list of sensors (should be equal to the list of SensorsRegistry
enum ESensors {
  CollisionSensor,
  DepthCamera,
  NormalsCamera,
  DVSCamera,
  GnssSensor,
  InertialMeasurementUnit,
  LaneInvasionSensor,
  ObstacleDetectionSensor,
  OpticalFlowCamera,
  Radar,
  RayCastSemanticLidar,
  RayCastLidar,
  RssSensor,
  SceneCaptureCamera,
  SemanticSegmentationCamera,
  InstanceSegmentationCamera,
  WorldObserver,
  CameraGBufferUint8,
  CameraGBufferFloat,
  HSSLidar
};

bool ROS2::Enable(bool enable, Middleware middleware, int domain_id) {
  {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    if (enable) {
      auto resolve = MiddlewareFactory::ResolveMiddleware(middleware);
      if (!resolve.success) {
        log_error("ROS2: middleware '", MiddlewareToString(middleware),
            "' is not compiled into this binary. ROS2 is DISABLED.");
        return false;
      }
      MiddlewareFactory::SetMiddleware(middleware);
      // Configure the domain id before any transport context is created (the
      // shared participants are created lazily on first publisher/subscriber).
      MiddlewareConfig::SetDomainId(domain_id);
      const ResolvedDomainId resolved = MiddlewareConfig::ResolveEffective();
      const char* domain_source =
          (resolved.source == DomainIdSource::CommandLine)  ? "--ros-domain-id"
          : (resolved.source == DomainIdSource::Environment) ? "ROS_DOMAIN_ID"
                                                             : "default";
      log_info("ROS2: using middleware: ",
          MiddlewareToString(middleware), ", domain id: ", resolved.id,
          " (", domain_source, ")");
      _clock_publisher = std::make_shared<CarlaClockPublisher>();
    }
    _enabled = enable;
  }
  if (enable) {
    StartHmcFeedbackScheduler();
  } else {
    StopHmcFeedbackScheduler();
  }
  return true;
}

void ROS2::SetFrame(uint64_t frame) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  _frame = frame;

  for (auto& element : _subscribers) {
    auto actor = element.first;
    auto subscriber = element.second;
    auto cb_it = _actor_callbacks.find(actor);
    if (cb_it == _actor_callbacks.end()) continue;
    auto callback = cb_it->second;

    subscriber->ProcessMessages(callback);
  }
}

void ROS2::SetTimestamp(double timestamp) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  double integral;
  const double fractional = modf(timestamp, &integral);
  const double multiplier = 1000000000.0;
  _seconds = static_cast<int32_t>(integral);
  _nanoseconds = static_cast<uint32_t>(fractional * multiplier);

  _clock_publisher->Write(_seconds, _nanoseconds);
  _clock_publisher->Publish();

  // Refresh actor-owned state on the game thread. DDS publication itself is
  // performed by the independent virtual-VCU scheduler.
  for (auto& pair : _hmc_feedback_callbacks) {
    pair.second();
  }

  // Physical status: publish at 100 ms measured against the simulation timestamp.
  // Publish on first timestamp, after 100 ms elapsed, or on time rewind.
  const int64_t current_timestamp_ns =
      static_cast<int64_t>(_seconds) * 1000000000LL + static_cast<int64_t>(_nanoseconds);
  const int64_t elapsed_ns = current_timestamp_ns - _last_ego_vehicle_physical_status_timestamp_ns;
  const bool should_publish_physical_status =
      (_last_ego_vehicle_physical_status_timestamp_ns < 0) ||
      (elapsed_ns >= kVehiclePhysicalStatusPeriodNs) ||
      (elapsed_ns < kVehiclePhysicalStatusRewindThresholdNs);
  if (should_publish_physical_status) {
    if (!_ego_vehicle_physical_status_publisher) {
      _ego_vehicle_physical_status_publisher = std::make_shared<CarlaVehiclePhysicalStatusPublisher>();
    }
    for (auto& pair : _ego_vehicle_physical_status_callbacks) {
      pair.second();
    }
    _last_ego_vehicle_physical_status_timestamp_ns = current_timestamp_ns;
  }
}

void ROS2::GetTimestamp(int32_t& sec, uint32_t& nsec) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  sec = _seconds;
  nsec = _nanoseconds;
}

void ROS2::PublishHmcFeedback(
    void* actor,
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
    uint8_t gear_sel_ready,
    uint8_t stop_hold_ready) {
  static bool env_initialized = false;
  static uint8_t env_lng_ready = 1u;
  static uint8_t env_lat_ready = 1u;
  static uint8_t env_gear_ready = 1u;
  if (!env_initialized) {
    env_lng_ready = GetEnvReady("CARLA_HMC_LNG_CTRL_READY");
    env_lat_ready = GetEnvReady("CARLA_HMC_LAT_CTRL_READY");
    env_gear_ready = GetEnvReady("CARLA_HMC_GEAR_SEL_READY");
    env_initialized = true;
  }

  std::lock_guard<std::recursive_mutex> lock(_mutex);
  if (!_enabled || _actor_callbacks.find(actor) == _actor_callbacks.end()) {
    return;
  }
  _hmc_feedback_snapshots[actor] = HmcFeedbackSnapshot{
      aps_pct,
      bps_pct,
      actual_speed_kmh,
      target_speed_echo_kmh,
      actual_swa_deg,
      target_swa_echo_deg,
      lng_op_mode,
      lat_op_mode,
      actuator_fault,
      env_lng_ready == 1u ? lng_ctrl_ready : 0u,
      env_lat_ready == 1u ? lat_ctrl_ready : 0u,
      env_gear_ready == 1u ? gear_sel_ready : 0u,
      stop_hold_ready};
}

void ROS2::StartHmcFeedbackScheduler() {
  if (_hmc_feedback_scheduler_running.exchange(true)) {
    return;
  }
  _hmc_feedback_scheduler = std::thread([this]() { RunHmcFeedbackScheduler(); });
}

void ROS2::StopHmcFeedbackScheduler() {
  if (!_hmc_feedback_scheduler_running.exchange(false)) {
    return;
  }
  if (_hmc_feedback_scheduler.joinable()) {
    _hmc_feedback_scheduler.join();
  }
}

void ROS2::RunHmcFeedbackScheduler() {
  uint8_t alive_counter = 0u;
  auto next_publish = std::chrono::steady_clock::now();
  while (_hmc_feedback_scheduler_running.load()) {
    std::shared_ptr<HmcFeedbackPublisher> publisher;
    std::vector<HmcFeedbackSnapshot> snapshots;
    {
      // The game thread owns actor access and can hold _mutex while a frame is
      // being processed. Copy its POD snapshots, then keep DDS writes outside
      // that lock so a delayed game frame cannot delay the VCU heartbeat.
      std::lock_guard<std::recursive_mutex> lock(_mutex);
      if (_enabled && !_hmc_feedback_snapshots.empty()) {
        if (!_hmc_feedback_publisher) {
          _hmc_feedback_publisher = std::make_shared<HmcFeedbackPublisher>();
        }
        publisher = _hmc_feedback_publisher;
        snapshots.reserve(_hmc_feedback_snapshots.size());
        for (const auto& pair : _hmc_feedback_snapshots) {
          snapshots.push_back(pair.second);
        }
      }
    }
    for (const auto& feedback : snapshots) {
      publisher->Write(
          alive_counter++,
          feedback.aps_pct,
          feedback.bps_pct,
          feedback.actual_speed_kmh,
          feedback.target_speed_echo_kmh,
          feedback.actual_swa_deg,
          feedback.target_swa_echo_deg,
          feedback.lng_op_mode,
          feedback.lat_op_mode,
          feedback.actuator_fault,
          feedback.lng_ctrl_ready,
          feedback.lat_ctrl_ready,
          feedback.gear_sel_ready,
          feedback.stop_hold_ready);
      publisher->Publish();
    }
    next_publish += kHmcFeedbackPeriod;
    const auto now = std::chrono::steady_clock::now();
    if (next_publish < now) {
      next_publish = now;
    }
    std::this_thread::sleep_until(next_publish);
  }
}

void ROS2::PublishVehiclePhysicalStatus(
    void *actor,
    const std::string& frame_id,
    bool brake_status,
    bool abs_status,
    bool tcs_status,
    bool esc_status,
    uint8_t current_gear,
    float yaw_rate_radps,
    float lateral_acceleration_mps2,
    float longitudinal_acceleration_mps2,
    float steering_wheel_angle_deg,
    float vehicle_width_m,
    float vehicle_length_m,
    float vehicle_speed_kmh,
    float wheel_angular_velocity_fl_radps,
    float wheel_angular_velocity_fr_radps,
    float wheel_angular_velocity_rl_radps,
    float wheel_angular_velocity_rr_radps,
    bool ignition_status,
    uint64_t valid_fields) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  if (!_enabled || _actor_callbacks.find(actor) == _actor_callbacks.end()) {
    return;
  }
  if (!_ego_vehicle_physical_status_publisher) {
    _ego_vehicle_physical_status_publisher = std::make_shared<CarlaVehiclePhysicalStatusPublisher>();
  }
  _ego_vehicle_physical_status_publisher->Write(
      _seconds, _nanoseconds, frame_id,
      brake_status, abs_status, tcs_status, esc_status,
      current_gear,
      yaw_rate_radps,
      lateral_acceleration_mps2,
      longitudinal_acceleration_mps2,
      steering_wheel_angle_deg,
      vehicle_width_m,
      vehicle_length_m,
      vehicle_speed_kmh,
      wheel_angular_velocity_fl_radps,
      wheel_angular_velocity_fr_radps,
      wheel_angular_velocity_rl_radps,
      wheel_angular_velocity_rr_radps,
      ignition_status,
      valid_fields);
  _ego_vehicle_physical_status_publisher->Publish();
}

void ROS2::RegisterVehiclePhysicalStatusCallback(
    void* actor, ROS2::VehiclePhysicalStatusCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  if (!actor || !callback) return;
  _ego_vehicle_physical_status_callbacks[actor] = std::move(callback);
}

void ROS2::RegisterActor(void *actor, std::string ros_name, std::string frame_id, bool publish_tf) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  _registered_actors.insert({actor, ros_name});
  _frame_ids.insert({actor, frame_id});
  _tfs.insert({actor, publish_tf});
}

void ROS2::UnregisterActor(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  _registered_actors.erase(actor);
  _frame_ids.erase(actor);
  _actor_parent_map.erase(actor);
  _tfs.erase(actor);
}

void ROS2::RegisterActorParent(void *actor, void *parent) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  _actor_parent_map.insert({actor, parent});
}

void ROS2::RegisterSensor(void *actor, std::string ros_name, std::string frame_id, bool publish_tf) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  RegisterActor(actor, ros_name, frame_id, publish_tf);
}

void ROS2::UnregisterSensor(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  UnregisterActor(actor);
  _publishers.erase(actor);
}

void ROS2::RegisterVehicle(void *actor, std::string ros_name, std::string frame_id, ActorCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  RegisterActor(actor, ros_name, frame_id);

  // Register actor callback
  _actor_callbacks.insert({actor, std::move(callback)});

  // Register subscribers
  auto base_topic_name = GetActorBaseTopicName(actor);

  auto _vehicle_control_subscriber = std::make_shared<CarlaEgoVehicleControlSubscriber>(actor, base_topic_name, frame_id);
  _subscribers.insert({actor, _vehicle_control_subscriber});

  auto _ackermann_control_subscriber = std::make_shared<AckermannControlSubscriber>(actor, base_topic_name, frame_id);
  _subscribers.insert({actor, _ackermann_control_subscriber});

  // HMC AD-01/AD-02 command subscriber for the HMC control loop.
  // Uses fixed HMC topic names rather than the per-actor rt/carla namespace.
  auto _hmc_command_subscriber = std::make_shared<HmcCommandSubscriber>(actor, base_topic_name, frame_id);
  _subscribers.insert({actor, _hmc_command_subscriber});
}


void ROS2::UnregisterVehicle(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  UnregisterActor(actor);
  _actor_callbacks.erase(actor);
  _subscribers.erase(actor);
  _hmc_feedback_callbacks.erase(actor);
  _hmc_feedback_snapshots.erase(actor);
  _ego_vehicle_physical_status_callbacks.erase(actor);
}

void ROS2::RegisterHmcFeedbackCallback(void* actor, ROS2::HmcFeedbackCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  if (!actor || !callback) return;
  _hmc_feedback_callbacks[actor] = std::move(callback);
}

std::string ROS2::GetActorRosName(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _registered_actors.find(actor);
  return it != _registered_actors.end() ? it->second : "";
}

std::string ROS2::GetActorBaseTopicName(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _actor_parent_map.find(actor);
  if (it != _actor_parent_map.end()) {
    return GetActorBaseTopicName(it->second) + "/" + GetActorRosName(actor);
  } else {
    return "rt/carla/" + GetActorRosName(actor);
  }
}

std::string ROS2::GetFrameId(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _frame_ids.find(actor);
  return it != _frame_ids.end() ? it->second : "";
}

std::string ROS2::GetParentFrameId(void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _actor_parent_map.find(actor);
  if (it != _actor_parent_map.end()) {
    return GetFrameId(it->second);
  } else {
    return "map";
  }
}

std::shared_ptr<CarlaTransformPublisher> ROS2::GetOrCreateTransformPublisher(void *actor) {

  auto it = _tfs.find(actor);
  if (it == _tfs.end() || it->second == false) {
    return nullptr;
  }

  // Check if the transform publisher is already created
  auto itp = _tf_publishers.find(actor);
  if (itp != _tf_publishers.end()) {
    return itp->second;
  }

  auto tf_publisher = std::make_shared<CarlaTransformPublisher>();
  _tf_publishers.insert({actor, tf_publisher});
  return tf_publisher;
}

std::shared_ptr<BasePublisher> ROS2::GetOrCreateSensor(int type, void* actor) {

  // Check if the sensor publisher is already created
  auto it = _publishers.find(actor);
  if (it != _publishers.end()) {
    return it->second;
  }

  auto create_and_register = [&](auto publisher) {
    _publishers.insert({actor, publisher});
    return publisher;
  };

  std::string topic_name = GetActorBaseTopicName(actor);
  std::string frame_id = GetFrameId(actor);

  switch(type) {
    case ESensors::CollisionSensor:
      return create_and_register(std::make_shared<CarlaCollisionPublisher>(topic_name, frame_id));
    case ESensors::DepthCamera:
      return create_and_register(std::make_shared<CarlaDepthCameraPublisher>(topic_name, frame_id));
    case ESensors::NormalsCamera:
      return create_and_register(std::make_shared<CarlaNormalsCameraPublisher>(topic_name, frame_id));
    case ESensors::DVSCamera:
      return create_and_register(std::make_shared<CarlaDVSPublisher>(topic_name, frame_id));
    case ESensors::GnssSensor:
      return create_and_register(std::make_shared<CarlaGNSSPublisher>(topic_name, frame_id));
    case ESensors::InertialMeasurementUnit:
      return create_and_register(std::make_shared<CarlaIMUPublisher>(topic_name, frame_id));
    case ESensors::OpticalFlowCamera:
      return create_and_register(std::make_shared<CarlaOpticalFlowCameraPublisher>(topic_name, frame_id));
    case ESensors::Radar:
      return create_and_register(std::make_shared<CarlaRadarPublisher>(topic_name, frame_id));
    case ESensors::RayCastSemanticLidar:
      return create_and_register(std::make_shared<CarlaSemanticLidarPublisher>(topic_name, frame_id));
    case ESensors::RayCastLidar:
      return create_and_register(std::make_shared<CarlaLidarPublisher>(topic_name, frame_id));
    case ESensors::SceneCaptureCamera:
      return create_and_register(std::make_shared<CarlaRGBCameraPublisher>(topic_name, frame_id));
    case ESensors::SemanticSegmentationCamera:
      return create_and_register(std::make_shared<CarlaSSCameraPublisher>(topic_name, frame_id));
    case ESensors::InstanceSegmentationCamera:
       return create_and_register(std::make_shared<CarlaISCameraPublisher>(topic_name, frame_id));
    case ESensors::LaneInvasionSensor:
    case ESensors::ObstacleDetectionSensor:
    case ESensors::RssSensor:
    case ESensors::WorldObserver:
    case ESensors::CameraGBufferUint8:
    case ESensors::CameraGBufferFloat:
      return nullptr;
    case ESensors::HSSLidar:
      return create_and_register(std::make_shared<CarlaLidarPublisher>(topic_name, frame_id));
  }
  return nullptr;
}

void ROS2::ProcessDataFromCamera(
    uint64_t sensor_type,
    const carla::geom::Transform sensor_transform,
    const carla::SharedBufferView buffer,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(static_cast<int>(sensor_type), actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaCameraPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
    reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
  if (!header)
    return;

  sensor_publisher->WriteCameraInfo(_seconds, _nanoseconds, 0, 0, header->height, header->width, header->fov_angle, true);
  sensor_publisher->WriteImage(_seconds, _nanoseconds, header->height, header->width, reinterpret_cast<const uint8_t*>(buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromGNSS(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    const carla::geom::GeoLocation &data,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::GnssSensor, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaGNSSPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  sensor_publisher->Write(_seconds, _nanoseconds, data);
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromIMU(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    carla::geom::Vector3D accelerometer,
    carla::geom::Vector3D gyroscope,
    float compass,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::InertialMeasurementUnit, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaIMUPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  sensor_publisher->Write(_seconds, _nanoseconds, accelerometer, gyroscope, compass);
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromDVS(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    const carla::SharedBufferView buffer,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::DVSCamera, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaDVSPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
    reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
  if (!header)
    return;

  const size_t elements =  (buffer->size() - carla::sensor::s11n::ImageSerializer::header_offset) / sizeof(carla::sensor::data::DVSEvent);
  const size_t im_width = header->width;
  const size_t im_height = header->height;

  sensor_publisher->WriteCameraInfo(_seconds, _nanoseconds, 0, 0, static_cast<uint32_t>(im_height), static_cast<uint32_t>(im_width), header->fov_angle, true);
  sensor_publisher->WriteImage(_seconds, _nanoseconds, static_cast<uint32_t>(elements), header->height, header->width, reinterpret_cast<const uint8_t*>(buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
  sensor_publisher->WritePointCloud(_seconds, _nanoseconds, 1, static_cast<uint32_t>(elements), const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset)));
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromLidar(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    carla::sensor::data::LidarData &data,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::RayCastLidar, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaLidarPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  // The lidar returns a flat list of floats rather than structured detection points.
  // Each lidar detection consists of 4 floats: x, y, z, and intensity.
  // Divide the total number of floats by 4 to get the number of lidar detections.
  const uint32_t width = static_cast<uint32_t>(data._points.size() / 4);
  const uint32_t height = 1;
  sensor_publisher->WritePointCloud(_seconds, _nanoseconds, height, width, reinterpret_cast<uint8_t*>(data._points.data()));
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromSemanticLidar(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    carla::sensor::data::SemanticLidarData &data,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::RayCastSemanticLidar, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaSemanticLidarPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  const uint32_t width = static_cast<uint32_t>(data._ser_points.size());
  const uint32_t height = 1;
  sensor_publisher->WritePointCloud(_seconds, _nanoseconds, height, width, reinterpret_cast<uint8_t*>(data._ser_points.data()));
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromRadar(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    const carla::sensor::data::RadarData &data,
    void *actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::Radar, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaRadarPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  const uint32_t width = static_cast<uint32_t>(data.GetDetectionCount());
  const uint32_t height = 1;
  sensor_publisher->WritePointCloud(_seconds, _nanoseconds, height, width, reinterpret_cast<uint8_t*>(const_cast<carla::sensor::data::RadarDetection*>(data._detections.data())));
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromObstacleDetection(
    uint64_t sensor_type,
    const carla::geom::Transform /*sensor_transform*/,
    AActor * /*first_ctor*/,
    AActor * /*second_actor*/,
    float distance,
    void * /*actor*/) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  log_info("Sensor ObstacleDetector to ROS data: frame.", _frame, "sensor.", sensor_type, "distance.", distance);
}

void ROS2::ProcessDataFromCollisionSensor(
    uint64_t /*sensor_type*/,
    const carla::geom::Transform sensor_transform,
    uint32_t other_actor,
    carla::geom::Vector3D impulse,
    void* actor) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto base_publisher = GetOrCreateSensor(ESensors::CollisionSensor, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaCollisionPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  sensor_publisher->Write(_seconds, _nanoseconds, other_actor, impulse);
  sensor_publisher->Publish();

  if (transform_publisher) {
    transform_publisher->Write(_seconds, _nanoseconds, GetParentFrameId(actor), GetFrameId(actor), sensor_transform);
    transform_publisher->Publish();
  }
}

void ROS2::ProcessDataFromMap(const std::string &open_drive) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  if (!_enabled) {
    return;
  }
  if (open_drive.empty()) {
    // Reached once per episode start, never per frame, so logging
    // unconditionally cannot flood the output.
    log_warning("ROS2: empty OpenDRIVE description, skipping map publish");
    return;
  }
  // HMC workaround: skip the latched OpenDRIVE map publish when using the
  // CycloneDDS middleware. The CarlaMapPublisher currently crashes in
  // ddsi_serdata_init because the CycloneDDS sertype initialization path in
  // the vendored 0.10.5 build does not match the custom CDR passthrough
  // assumptions. The map topic is not required for the HMC control loop.
  if (MiddlewareFactory::GetMiddleware() == Middleware::CycloneDDS) {
    log_info("ROS2: skipping OpenDRIVE map publish under CycloneDDS (HMC workaround)");
    return;
  }
  if (!_map_publisher) {
    _map_publisher = std::make_shared<CarlaMapPublisher>();
  }
  _map_publisher->Write(open_drive);
  _map_publisher->Publish();
}

void ROS2::Shutdown() {
  StopHmcFeedbackScheduler();
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  // Destroy publishers first so DataWriter unregister-dispose messages are
  // sent while the shared DomainParticipant is still alive, then clear
  // subscribers.  Order matters: the FastDDS shared participant is refcounted;
  // destroying all endpoints (publishers and subscribers) drives the refcount
  // to zero and triggers delete_contained_entities() + delete_participant().
  _publishers.clear();
  _tf_publishers.clear();
  _clock_publisher.reset();
  _map_publisher.reset();
  _hmc_feedback_publisher.reset();
  _hmc_feedback_snapshots.clear();
  _ego_vehicle_physical_status_publisher.reset();

  _subscribers.clear();

  _enabled = false;
}

} // namespace ros2
} // namespace carla
