// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Buffer.h"
#include "carla/BufferView.h"
#include "carla/geom/Transform.h"
#include "carla/ros2/ROS2CallbackData.h"
#include "carla/ros2/middleware/Middleware.h"
#include "carla/ros2/middleware/MiddlewareConfig.h"
#include "carla/streaming/detail/Types.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <vector>

// forward declarations
class AActor;
namespace carla {
  namespace geom {
    class GeoLocation;
    class Vector3D;
  }
  namespace sensor {
    namespace data {
      struct DVSEvent;
      class LidarData;
      class SemanticLidarData;
      class RadarData;
    }
  }
}
namespace carla {
namespace ros2 {

  class BasePublisher;
  class BaseSubscriber;

  class CarlaTransformPublisher;
  class CarlaClockPublisher;
  class CarlaMapPublisher;
  class HmcFeedbackPublisher;
  class CarlaVehiclePhysicalStatusPublisher;

class ROS2
{
  public:

    // deleting copy constructor for singleton
    ROS2(const ROS2& obj) = delete;
    static std::shared_ptr<ROS2> GetInstance() {
      if (!_instance)
        _instance = std::shared_ptr<ROS2>(new ROS2);
      return _instance;
    }

    // General
    // Returns true when enabling succeeds (middleware compiled in), false otherwise.
    // Callers pass enable=false to shut down; the return value is always true in that case.
    // domain_id selects the ROS 2 domain id for the chosen middleware; kUnsetDomainId
    // (the default) keeps each middleware's native default.
    bool Enable(bool enable, Middleware middleware = Middleware::CycloneDDS,
        int domain_id = kUnsetDomainId);
    void Shutdown();

    bool IsEnabled() { return _enabled; }

    void SetFrame(uint64_t frame);
    void SetTimestamp(double timestamp);
    void GetTimestamp(int32_t& sec, uint32_t& nsec) const;

    std::string GetActorRosName(void *actor);
    std::string GetActorBaseTopicName(void *actor);

    std::string GetFrameId(void *actor);
    std::string GetParentFrameId(void *actor);

    // Registration
    void RegisterActor(void *actor, std::string ros_name, std::string frame_id, bool publish_tf=true);
    void UnregisterActor(void *actor);

    void RegisterActorParent(void *actor, void *parent);

    void RegisterSensor(void *actor, std::string ros_name, std::string frame_id, bool publish_tf);
    void UnregisterSensor(void *actor);

    void RegisterVehicle(void *actor, std::string ros_name, std::string frame_id, ActorCallback callback);
    void UnregisterVehicle(void *actor);

    // Register a callback that refreshes the HMC feedback snapshot from the
    // UE4 game thread. A separate virtual-VCU scheduler sends that snapshot.
    using HmcFeedbackCallback = std::function<void()>;
    void RegisterHmcFeedbackCallback(void* actor, HmcFeedbackCallback callback);

    // Register a callback that publishes the atomic ego-vehicle physical status.
    // ROS2::SetTimestamp invokes it at the configured physical-status period
    // (100 ms) independently of the HMC feedback period.
    using VehiclePhysicalStatusCallback = std::function<void()>;
    void RegisterVehiclePhysicalStatusCallback(
        void* actor, VehiclePhysicalStatusCallback callback);

    // Refresh the HMC FB-01 snapshot from the UE4 game thread where the
    // ACarlaWheeledVehicle pointer is valid. The scheduler owns DDS writes.
    void PublishHmcFeedback(
        void *actor,
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
        uint8_t stop_hold_ready);

    // Publish the atomic ego-vehicle physical status snapshot.
    void PublishVehiclePhysicalStatus(
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
        uint64_t valid_fields);

    // Receiving data to publish
    void ProcessDataFromCamera(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      const carla::SharedBufferView buffer,
      void *actor = nullptr);
    void ProcessDataFromGNSS(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      const carla::geom::GeoLocation &data,
      void *actor = nullptr);
    void ProcessDataFromIMU(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      carla::geom::Vector3D accelerometer,
      carla::geom::Vector3D gyroscope,
      float compass,
      void *actor = nullptr);
    void ProcessDataFromDVS(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      const carla::SharedBufferView buffer,
      void *actor = nullptr);
    void ProcessDataFromLidar(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::LidarData &data,
      void *actor = nullptr);
    void ProcessDataFromSemanticLidar(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::SemanticLidarData &data,
      void *actor = nullptr);
    void ProcessDataFromRadar(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      const carla::sensor::data::RadarData &data,
      void *actor = nullptr);
    void ProcessDataFromObstacleDetection(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      AActor *first_actor,
      AActor *second_actor,
      float distance,
      void *actor = nullptr);
    void ProcessDataFromCollisionSensor(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      uint32_t other_actor,
      carla::geom::Vector3D impulse,
      void* actor);
    // Publishes the OpenDRIVE description of the current map as a latched
    // topic. Called once per episode; re-publishing refreshes the latched
    // sample after a map change.
    void ProcessDataFromMap(const std::string &open_drive);

  private:
    struct HmcFeedbackSnapshot {
      float aps_pct {0.0f};
      float bps_pct {0.0f};
      float actual_speed_kmh {0.0f};
      float target_speed_echo_kmh {0.0f};
      float actual_swa_deg {0.0f};
      float target_swa_echo_deg {0.0f};
      uint8_t lng_op_mode {0u};
      uint8_t lat_op_mode {0u};
      bool actuator_fault {false};
      uint8_t lng_ctrl_ready {0u};
      uint8_t lat_ctrl_ready {0u};
      uint8_t gear_sel_ready {0u};
      uint8_t stop_hold_ready {0u};
    };

    void StartHmcFeedbackScheduler();
    void StopHmcFeedbackScheduler();
    void RunHmcFeedbackScheduler();
    std::shared_ptr<CarlaTransformPublisher> GetOrCreateTransformPublisher(void *actor);
    std::shared_ptr<BasePublisher> GetOrCreateSensor(int type, void* actor);

  // sigleton
  ROS2() {};

  static std::shared_ptr<ROS2> _instance;

  // Protects all map members from concurrent access by the UE4 tick thread
  // (ProcessDataFrom*, SetFrame) and the RPC thread (Register*, Unregister*).
  // recursive_mutex is required because RegisterSensor calls RegisterActor,
  // and UnregisterSensor calls UnregisterActor.
  mutable std::recursive_mutex _mutex;

  bool _enabled { false };
  uint64_t _frame { 0 };
  int32_t _seconds { 0 };
  uint32_t _nanoseconds { 0 };

  std::shared_ptr<CarlaClockPublisher> _clock_publisher;
  std::shared_ptr<CarlaMapPublisher> _map_publisher;
  std::shared_ptr<HmcFeedbackPublisher> _hmc_feedback_publisher;
  std::shared_ptr<CarlaVehiclePhysicalStatusPublisher> _ego_vehicle_physical_status_publisher;

  // Game-thread snapshots and their independent virtual-VCU transport loop.
  std::unordered_map<void*, HmcFeedbackCallback> _hmc_feedback_callbacks;
  std::unordered_map<void*, HmcFeedbackSnapshot> _hmc_feedback_snapshots;
  std::atomic<bool> _hmc_feedback_scheduler_running {false};
  std::thread _hmc_feedback_scheduler;
  static constexpr auto kHmcFeedbackPeriod = std::chrono::milliseconds(10);

  // Ego-vehicle physical status callbacks registered by hero vehicles.
  // Invoked from SetTimestamp at the configured physical-status period
  // (100 ms) measured against the simulation timestamp.
  std::unordered_map<void*, VehiclePhysicalStatusCallback> _ego_vehicle_physical_status_callbacks;
  int64_t _last_ego_vehicle_physical_status_timestamp_ns { -1 };
  static constexpr int64_t kVehiclePhysicalStatusPeriodNs = 100000000LL;  // 100 ms
  static constexpr int64_t kVehiclePhysicalStatusRewindThresholdNs = -1000000LL;  // -1 ms

  // actor->parent relationship
  std::unordered_map<void *, void *> _actor_parent_map;

  std::unordered_map<void *, std::string> _registered_actors;
  std::unordered_map<void *, std::string> _frame_ids;

  std::unordered_map<void *, std::shared_ptr<BasePublisher>> _publishers;
  std::unordered_multimap<void *, std::shared_ptr<BaseSubscriber>> _subscribers;
  std::unordered_map<void *, ActorCallback> _actor_callbacks;

  std::unordered_map<void *, bool> _tfs;
  std::unordered_map<void *, std::shared_ptr<CarlaTransformPublisher>> _tf_publishers;
};

} // namespace ros2
} // namespace carla
