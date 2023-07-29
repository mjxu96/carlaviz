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
#include <carlaviz/utils/sensor.h>
#include "base.h"

#include <xviz/xviz.h>
#include <nlohmann/json.hpp>

#include <unordered_map>

namespace carlaviz::translations {

template <typename ConnectorType>
class XVIZMetadataUpdater {
 public:
  XVIZMetadataUpdater(ConnectorType& connector, bool allow_static_objects)
      : connector_(connector), allow_static_objects_(allow_static_objects) {
    ReConstructMetadata();
  }

  void ReConstructMetadata() {
    metadata_builder_.Reset();
    metadata_builder_.Version("2.0.0");
    // clang-format off
    auto& builder = metadata_builder_
      // pose
      .Stream("/vehicle_pose")
        .template Category<xviz::StreamMetadata::POSE>()
      .Stream("/vehicle/acceleration")
        .template Category<xviz::StreamMetadata::TIME_SERIES>()
          .Unit("m/s^2")
          .Type(xviz::StreamMetadata::FLOAT)
      .Stream("/vehicle/velocity")
        .template Category<xviz::StreamMetadata::TIME_SERIES>()
          .Unit("m/s")
          .Type(xviz::StreamMetadata::FLOAT)
      .Stream("/game/info")
        .template Category<xviz::StreamMetadata::UI_PRIMITIVE>()
      .Stream("/vehicle/info")
        .template Category<xviz::StreamMetadata::UI_PRIMITIVE>()
      // vehicles
      .Stream("/object/vehicle")
        .template Category<xviz::StreamMetadata::PRIMITIVE>()
          .Type(xviz::StreamMetadata::POLYGON)
          .Coordinate(xviz::StreamMetadata::IDENTITY)
          .StreamStyle(
            {
              {"extruded", true},
              {"fill_color", "#ff66cc"}
            })
      // pedestrians
      .Stream("/object/people")
        .template Category<xviz::StreamMetadata::PRIMITIVE>()
          .Type(xviz::StreamMetadata::POLYGON)
          .Coordinate(xviz::StreamMetadata::IDENTITY)
          .StreamStyle(
            {
              {"extruded", true},
              {"fill_color", "#ff0000"}
            })
      // traffic lights indicators
      .Stream("/traffic/traffic_lights/indicators")
        .template Category<xviz::StreamMetadata::PRIMITIVE>()
          .Type(xviz::StreamMetadata::POLYGON)
          .Coordinate(xviz::StreamMetadata::IDENTITY)
          .StreamStyle(
            {
              {"extruded", true},
              {"height", 0.1f}
            })
          .StyleClass("red", {{"fill_color", "#ff0000"}})
          .StyleClass("yellow", {{"fill_color", "#ffff00"}})
          .StyleClass("green", {{"fill_color", "#00ff00"}})
          .StyleClass("unknown", {{"fill_color", "#c0c0c0"}})
      // traffic lights objects
      .Stream("/traffic/traffic_lights/objects")
        .template Category<xviz::StreamMetadata::PRIMITIVE>()
          .Type(xviz::StreamMetadata::POLYGON)
          .Coordinate(xviz::StreamMetadata::IDENTITY)
          .StreamStyle(
            {
              {"extruded", true},
              {"height", 0.1f}
            })
          .StyleClass("red", {{"fill_color", "#ff0000"}})
          .StyleClass("yellow", {{"fill_color", "#ffff00"}})
          .StyleClass("green", {{"fill_color", "#00ff00"}})
          .StyleClass("unknown", {{"fill_color", "#c0c0c0"}});
      
    // add cameras
    auto& builder_after_adding_cameras = AddSensorStream(xviz::StreamMetadata::IMAGE
    , builder, camera_streams_.begin(), camera_streams_);

    // add lidars
    auto& builder_after_adding_lidars = AddSensorStream(xviz::StreamMetadata::POINT, 
    builder_after_adding_cameras, lidar_streams_.begin(), lidar_streams_, {
      {"radius_pixels", 2u}
    });

    // add ui
    std::vector<std::string> camera_streams_vec;
    for (const auto& [_, stream_name] : camera_streams_) {
      camera_streams_vec.push_back(stream_name);
    }
    builder_after_adding_lidars.UI("Camera")
      .Container("Camera", xviz::LayoutType::HORIZONTAL)
        .Video(camera_streams_vec)
      .EndContainer()
    .UI("Tables")
      .Container("Tables", xviz::LayoutType::HORIZONTAL)
        .TreeTable("Simulator Info", "Simulator Info", "/game/info", false)
        .TreeTable("Ego Vehicle Info", "Ego Vehicle Info", "/vehicle/info", false)
      .EndContainer()
    .UI("Metrics")
      .Container("Metrics", xviz::LayoutType::VERTICAL)
        .Metric("Velocity", "Velocity of Ego Vehicle", {"/vehicle/velocity"})
        .Metric("Acceleration", "Acceleration of Ego Vehicle", {"/vehicle/acceleration"})
      .EndContainer();
    // clang-format on
    new_metadata_required_ = true;
  }

  void UpdateCamera(sensor::SensorIDType sensor_id,
                    const std::string& stream_name) {
    CheckAndUpdateSensorStream(sensor_id, stream_name, camera_streams_);
  }

  void UpdateLidar(sensor::SensorIDType sensor_id,
                   const std::string& stream_name) {
    CheckAndUpdateSensorStream(sensor_id, stream_name, lidar_streams_);
  }

  void RemoveSensor(sensor::SensorIDType sensor_id) {
    std::size_t removed = 0;
    removed += camera_streams_.erase(sensor_id);
    removed += lidar_streams_.erase(sensor_id);
    if (removed > 0) {
      logging::LogInfo("Removed sensor id {} in metadata, rebuilding metadata",
                       sensor_id);
      ReConstructMetadata();
    }
  }

  void UpdateMap(const map::Map& map) {
    map_ = map;
    new_metadata_required_ = true;
    logging::LogInfo("Map {} is updated in translation layer.", map_.Name());
  }

  void Reset() {
    ClearSensorStreams();
    map_.Reset();
    ReConstructMetadata();
    new_metadata_required_ = false;
  }

  void Process() {
    logging::LogDebug(
        "XVIZ MetadataUpdater Process() is called with new_metadata_required = "
        "{}",
        new_metadata_required_);
    if (new_metadata_required_) {
      new_metadata_required_ = false;
      auto& metadata = metadata_builder_.GetData();
      xviz::Message<xviz::Metadata> metadata_message(metadata);
      std::string metadata_json_string = metadata_message.ToJsonString();
      if (!map_.Empty()) {
        logging::LogInfo(
            "XVIZ MetadataUpdater metadata is updated with map with {} "
            "roads{}.",
            map_.Roads().size(),
            (allow_static_objects_ ? " and static objects" : ""));
        nlohmann::json metadata_json =
            nlohmann::json::parse(metadata_json_string);
        metadata_json["data"]["map"] = map_.ToGeoJson(allow_static_objects_);
        connector_.UpdateMetadata(metadata_json.dump());
      } else {
        logging::LogInfo(
            "XVIZ MetadataUpdater metadata is updated without map.");
        connector_.UpdateMetadata(std::move(metadata_json_string));
      }
    }
  }

 protected:
  ConnectorType& connector_;

  map::Map map_;
  xviz::MetadataBuilder metadata_builder_;
  bool new_metadata_required_ = false;

  bool allow_static_objects_ = true;

  // streams for sensors
  std::unordered_map<sensor::SensorIDType, std::string> camera_streams_;
  std::unordered_map<sensor::SensorIDType, std::string> lidar_streams_;

  void ClearSensorStreams() {
    camera_streams_.clear();
    lidar_streams_.clear();
  }

  template <typename BuilderType, typename IteratorType, typename ContainerType>
  auto& AddSensorStream(xviz::StreamMetadata::PrimitiveType type,
                        BuilderType& builder, IteratorType itr,
                        const ContainerType& container,
                        const xviz::StyleType& extra_style = {}) {
    if (itr == container.end()) {
      return builder;
    }
    auto old_itr = itr;
    itr++;
    auto& new_builder =
        builder.Stream(old_itr->second)
            .template Category<xviz::StreamMetadata::PRIMITIVE>()
            .Type(type);
    if (extra_style.empty()) {
      return AddSensorStream(type, new_builder, itr, container);
    } else {
      return AddSensorStream(type, new_builder.StreamStyle(extra_style), itr,
                             container);
    }
  }

  void CheckAndUpdateSensorStream(
      sensor::SensorIDType sensor_id, const std::string& stream_name,
      std::unordered_map<sensor::SensorIDType, std::string>& streams) {
    if (streams.find(sensor_id) == streams.end()) {
      streams.insert({sensor_id, stream_name});
      logging::LogInfo("Added one stream {} for sensor id {}", stream_name,
                       sensor_id);
      ReConstructMetadata();
    }
  }
};

template <typename CT>
struct XVIZTranslationTrait {
  using ConnectorType = CT;
  using MetadataUpdateType = XVIZMetadataUpdater<ConnectorType>;
};

template <typename ConnectorType>
class XVIZTranslation
    : public TranslationBase<XVIZTranslation<ConnectorType>,
                             XVIZTranslationTrait<ConnectorType>> {
 public:
  using BaseType = TranslationBase<XVIZTranslation<ConnectorType>,
                                   XVIZTranslationTrait<ConnectorType>>;
  using BaseType::BaseType;

  void UpdateTimeImpl(double now, uint32_t current_frame) {
    if (is_time_set_) {
      return;
    }
    builder_.Timestamp(now);

    builder_.UIPrimitive("/game/info")
        .Column("Simulator Time", xviz::TreeTableColumn::DOUBLE, "second")
        .Column("Simulator Frame", xviz::TreeTableColumn::INT32)
        .Row(0, {now, (int)current_frame});
    is_time_set_ = true;
  }

  void UpdatePoseImpl(double now, const std::array<float, 3>& position,
                      const std::array<float, 3>& orientation, double velocity,
                      double acceleration) {
    if (is_pose_set_) {
      return;
    }

    builder_.Pose("/vehicle_pose")
        .MapOrigin(0, 0, 0)
        .Position(position[0], position[1], position[2])
        .Orientation(orientation[0], orientation[1], orientation[2]);

    builder_.TimeSeries("/vehicle/velocity")
        .Timestamp(now)
        .Value(velocity)
        .ID("velocity")
        .TimeSeries("/vehicle/acceleration")
        .Timestamp(now)
        .Value(acceleration)
        .ID("acceleration");

    builder_.UIPrimitive("/vehicle/info")
        .Column("Position", xviz::TreeTableColumn::STRING)
        .Column("Orientation", xviz::TreeTableColumn::STRING)
        .Row(0, {fmt::format("[{}]", fmt::join(position, ", ")),
                 fmt::format("[{}]", fmt::join(orientation, ", "))});
    is_pose_set_ = true;
  }
  void UpdateVehicleImpl(const std::string& object_id,
                         const std::vector<std::array<float, 3>>& vertices,
                         float height) {
    builder_.Primitive("/object/vehicle")
        .Polygon(vertices)
        .ID(object_id)
        .Style({{"height", height}});
  }

  void UpdatePeopleImpl(const std::string& object_id,
                        const std::vector<std::array<float, 3>>& vertices,
                        float height) {
    builder_.Primitive("/object/people")
        .Polygon(vertices)
        .ID(object_id)
        .Style({{"height", height}});
  }

  void UpdateImageImpl(const sensor::Image& image) {
    std::string stream_name = "/sensor/camera/" + image.info.name + "/" +
                              std::to_string(image.info.id);
    this->metadata_updater_.UpdateCamera(image.info.id, stream_name);
    builder_.Primitive(stream_name)
        .Image(image.data.data(), image.data.size())
        .ID(std::to_string(image.info.id));
  }

  void UpdateLidarMeasurementImpl(
      const sensor::LidarMeasurement& lidar_measurement) {
    std::string stream_name = "/sensor/lidar/" + lidar_measurement.info.name +
                              "/" + std::to_string(lidar_measurement.info.id);
    this->metadata_updater_.UpdateLidar(lidar_measurement.info.id, stream_name);
    if (!lidar_measurement.colors.empty()) {
      builder_.Primitive(stream_name)
          .Point(lidar_measurement.points)
          .Color(lidar_measurement.colors)
          .ID(std::to_string(lidar_measurement.info.id));
    } else {
      builder_.Primitive(stream_name)
          .Point(lidar_measurement.points)
          .ID(std::to_string(lidar_measurement.info.id));
    }
  }

  void UpdateTrafficLightImpl(uint64_t id, map::TrafficLightStatus status) {
    auto traffic_lights_itr = traffic_lights_.find(id);
    if (traffic_lights_itr == traffic_lights_.end()) [[unlikely]] {
      logging::LogError(
          "Traffic lights {} was not created before, something must be wrong",
          id);
      return;
    }

    builder_.Primitive("/traffic/traffic_lights/indicators")
        .Polygon(map::PointsVectorToArrayVector(
            traffic_lights_itr->second.affected_regions))
        .Style({{"height", 0.1f}})
        .Classes({TrafficLightStatusToString(status)});

    if (this->option_.allow_static_objects) {
      for (const auto& light : traffic_lights_itr->second.lights) {
        builder_.Primitive("/traffic/traffic_lights/objects")
            .Polygon(map::PointsVectorToArrayVector(light.vertices))
            .Style({{"height", light.height}})
            .Classes({TrafficLightStatusToString(status)});
      }
    }
  }

  void RemoveSensorImpl(sensor::SensorIDType sensor_id) {
    this->metadata_updater_.RemoveSensor(sensor_id);
  }

  void ProcessImpl() {
    if (!is_time_set_) {
      this->UpdateTime(0, 0);
    }
    if (!is_pose_set_) {
      this->UpdatePose(0, {0, 0, 0}, {0, 0, 0}, 0, 0);
    }

    this->connector_.UpdateSnapshot(
        xviz::Message<xviz::StateUpdate>(builder_.GetData())
            .ToProtobufBinary());

    // reset builder every time
    builder_.Reset();
    is_time_set_ = false;
    is_pose_set_ = false;
  }

  void UpdateMapImpl(const map::Map& map) {
    // record positions of traffic lights
    for (const auto& traffic_light : map.TrafficLights()) {
      traffic_lights_.insert({traffic_light.id, traffic_light});
    }
  }

  void ResetImpl() {
    builder_.Reset();
    traffic_lights_.clear();
  }

 protected:
  xviz::Builder builder_;

  bool is_pose_set_ = false;
  bool is_time_set_ = false;

  std::unordered_map<uint64_t, map::TrafficLight> traffic_lights_;
};

}  // namespace carlaviz::translations
