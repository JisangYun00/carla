// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "carla/ros2/subscribers/BaseSubscriber.h"
#include "carla/ros2/middleware/MiddlewareFactory.h"
#include "carla/Logging.h"

namespace carla {
namespace ros2 {

  template<typename S>
  class SubscriberImpl {
  public:
    using msg_type = typename S::msg_type;

    bool Init(std::string topic_name) {
#ifdef LIBCARLA_WITH_GTEST
      if (!_middleware) {
#endif
        _middleware = MiddlewareFactory::CreateSubscriber<S>();
        if (!_middleware) {
          log_error("SubscriberImpl: Failed to create middleware subscriber");
          return false;
        }
#ifdef LIBCARLA_WITH_GTEST
      }
#endif
      return _middleware->Init(
          topic_name, &_message, &_new_message, &_message_mutex);
    }

    std::string GetTopicName() {
      if (_middleware) {
        return _middleware->GetTopicName();
      }
      return "";
    }

    bool IsAlive() {
      if (_middleware) {
        return _middleware->IsAlive();
      }
      return false;
    }

    msg_type GetMessage() {
      std::lock_guard<std::mutex> lock(_message_mutex);
      _new_message = false;
      return _message;
    }

    bool HasNewMessage() {
      std::lock_guard<std::mutex> lock(_message_mutex);
      return _new_message;
    }

    bool TakeMessage(msg_type& message) {
      std::lock_guard<std::mutex> lock(_message_mutex);
      if (!_new_message) return false;
      message = _message;
      _new_message = false;
      return true;
    }

#ifdef LIBCARLA_WITH_GTEST
    void SetMiddlewareForTesting(std::unique_ptr<ISubscriberMiddleware> middleware) {
      _middleware = std::move(middleware);
    }

    void SimulateMessageReceiptForTesting(const msg_type& msg) {
      std::lock_guard<std::mutex> lock(_message_mutex);
      _message = msg;
      _new_message = true;
    }
#endif

  private:
    std::unique_ptr<ISubscriberMiddleware> _middleware;
    msg_type _message;
    bool _new_message { false };
    std::mutex _message_mutex;
  };

}  // namespace ros2
}  // namespace carla
