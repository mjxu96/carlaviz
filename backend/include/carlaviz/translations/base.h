/*
 * Project: carlaviz
 * Description: Carla Visulization in Browser
 * Author: Minjun Xu (mjxu96@outlook.com)
 * -----
 * MIT License
 * Copyright (c) 2023 Minjun Xu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#pragma once

#include <carlaviz/utils/definitions.h>
#include <carlaviz/utils/logging.h>
#include <carlaviz/utils/map.h>
#include <carlaviz/utils/options.h>

#include <cstdint>
#include <functional>

namespace carlaviz::translations {

using TranslationObjectId = uint32_t;

template <typename DerivedTranslation, typename TranslationTrait>
class TranslationBase {
  using ConnectorType = typename TranslationTrait::ConnectorType;
  using MetadataUpdateType = typename TranslationTrait::MetadataUpdateType;

 public:
  TranslationBase(utils::TranslationOption option, ConnectorType& connector)
      : option_(option),
        connector_(connector),
        metadata_updater_(connector, option_.allow_static_objects) {
    connector_.RegisterFrontendParameterUpdateCallback(
        std::bind(&TranslationBase::OnFrontendParameterUpdate, this,
                  std::placeholders::_1));
  }

  void RegisterSensorSubscriptionStateUpdateCallback(
      const utils::SensorSubscriptionStateUpdateCallbackType& callback) {
    sensor_subscription_state_update_callback_ = callback;
  }

  void OnFrontendParameterUpdate(const std::string&) {
    // to stop some data transmissions
  }

  void Reset() {
    connector_.Reset();
    metadata_updater_.Reset();
    return static_cast<DerivedTranslation*>(this)->ResetImpl();
  }

  void Process() {
    metadata_updater_.Process();
    return static_cast<DerivedTranslation*>(this)->ProcessImpl();
  }

  void UpdateMap(const map::Map& map) {
    metadata_updater_.UpdateMap(map);
    return static_cast<DerivedTranslation*>(this)->UpdateMapImpl(map);
  }

  void UpdateTime(double now, uint32_t current_frame) {
    return static_cast<DerivedTranslation*>(this)->UpdateTimeImpl(
        now, current_frame);
  }

  void UpdatePose(double now, const std::array<float, 3>& position,
                  const std::array<float, 3>& orientation, double velocity,
                  double acceleration) {
    return static_cast<DerivedTranslation*>(this)->UpdatePoseImpl(
        now, position, orientation, velocity, acceleration);
  }
  void UpdateVehicle(const std::string& object_id,
                     const std::vector<std::array<float, 3>>& vertices,
                     float height) {
    return static_cast<DerivedTranslation*>(this)->UpdateVehicleImpl(
        object_id, vertices, height);
  }

  void UpdatePeople(const std::string& object_id,
                    const std::vector<std::array<float, 3>>& vertices,
                    float height) {
    return static_cast<DerivedTranslation*>(this)->UpdatePeopleImpl(
        object_id, vertices, height);
  }

  void UpdateImage(const sensor::Image& image) {
    return static_cast<DerivedTranslation*>(this)->UpdateImageImpl(image);
  }

  void UpdateLidarMeasurement(
      const sensor::LidarMeasurement& lidar_measurement) {
    return static_cast<DerivedTranslation*>(this)->UpdateLidarMeasurementImpl(
        lidar_measurement);
  }

  void UpdateTrafficLight(uint64_t id, map::TrafficLightStatus status) {
    return static_cast<DerivedTranslation*>(this)->UpdateTrafficLightImpl(
        id, status);
  }

  void RemoveSensor(sensor::SensorIDType sensor_id) {
    return static_cast<DerivedTranslation*>(this)->RemoveSensorImpl(sensor_id);
  }

 protected:
  utils::TranslationOption option_;
  ConnectorType& connector_;
  MetadataUpdateType metadata_updater_;
  utils::SensorSubscriptionStateUpdateCallbackType
      sensor_subscription_state_update_callback_;
};

}  // namespace carlaviz::translations
