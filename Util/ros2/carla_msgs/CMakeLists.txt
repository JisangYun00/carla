cmake_minimum_required(VERSION 2.8.3)
project(carla_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(SERVICE_FILES SpawnObject.srv DestroyObject.srv GetBlueprints.srv RespawnVehicle.srv)

set(MSG_FILES
    CarlaBoundingBox.msg
    CarlaEgoVehicleControl.msg
    CarlaEgoVehicleInfoWheel.msg
    CarlaEgoVehicleInfo.msg
    CarlaVehiclePhysicalStatus.msg
    CarlaEgoVehicleStatus.msg
    CarlaCollisionEvent.msg
    CarlaLaneInvasionEvent.msg
    CarlaWorldInfo.msg
    CarlaActorInfo.msg
    CarlaActorList.msg
    CarlaControl.msg
    CarlaStatus.msg
    CarlaTrafficLightInfo.msg
    CarlaTrafficLightInfoList.msg
    CarlaTrafficLightStatus.msg
    CarlaTrafficLightStatusList.msg
    CarlaWalkerControl.msg
    CarlaWeatherParameters.msg)

cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Apend "msg/" to each file name
set(TEMP_LIST "")
foreach(MSG_FILE ${MSG_FILES})
  list(APPEND TEMP_LIST "msg/${MSG_FILE}")
endforeach()
set(MSG_FILES ${TEMP_LIST})

# Apend "srv/" to each file name
set(TEMP_LIST "")
foreach(SERVICE_FILE ${SERVICE_FILES})
  list(APPEND TEMP_LIST "srv/${SERVICE_FILE}")
endforeach()
set(SERVICE_FILES ${TEMP_LIST})

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  ${SERVICE_FILES}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  diagnostic_msgs
  ADD_LINTER_TESTS)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
