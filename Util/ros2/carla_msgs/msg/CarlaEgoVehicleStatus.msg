#
# Copyright (c) 2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

std_msgs/Header header

float32 velocity

float32 x
float32 y
float32 z

float32 vx
float32 vy
float32 vz

geometry_msgs/Accel acceleration
geometry_msgs/Quaternion orientation

# the current control values, as reported by Carla
CarlaEgoVehicleControl control
