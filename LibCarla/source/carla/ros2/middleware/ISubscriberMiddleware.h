// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <mutex>
#include <string>

namespace carla {
namespace ros2 {

/// Type-erased abstract interface for a subscriber middleware.
/// Concrete implementations write received messages directly into the caller-provided
/// storage (message_ptr / new_message_flag) to avoid an extra copy.
///
/// Contract: when an implementation establishes its process-wide transport
/// context (a DDS DomainParticipant, a Zenoh session, ...) it must honor
/// carla::ros2::MiddlewareConfig::GetDomainId(), mapping kUnsetDomainId to its
/// own native default. This keeps the ROS 2 domain id configurable and
/// middleware-agnostic (see MiddlewareConfig.h).
class ISubscriberMiddleware {
 public:
  virtual ~ISubscriberMiddleware() = default;

  /// Initialize the underlying middleware entities.
  /// The middleware writes incoming messages to *message_ptr and sets *new_message_flag = true.
  /// message_mutex serializes that write with SubscriberImpl consumption.
  /// @param topic_name       Full topic name.
  /// @param message_ptr      Pointer to the message storage owned by SubscriberImpl<S>.
  /// @param new_message_flag Pointer to the new-message flag owned by SubscriberImpl<S>.
  /// @param message_mutex    Mutex owned by SubscriberImpl<S> for the mailbox.
  /// @return true on success.
  virtual bool Init(
      const std::string& topic_name,
      void* message_ptr,
      bool* new_message_flag,
      std::mutex* message_mutex) = 0;

  /// @return true if at least one publisher is matched.
  virtual bool IsAlive() const = 0;

  /// @return The topic name this subscriber is bound to.
  virtual std::string GetTopicName() const = 0;
};

} // namespace ros2
} // namespace carla
