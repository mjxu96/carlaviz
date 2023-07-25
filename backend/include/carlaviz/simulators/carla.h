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

#include "base.h"

#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/Client.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/World.h>
#include <carla/client/WorldSnapshot.h>
#include <carla/image/CityScapesPalette.h>
#include <carla/rpc/ObjectLabel.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/SemanticLidarMeasurement.h>

#include <carlaviz/utils/utils.h>

#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace carlaviz::simulators {

template <typename TranslationType>
class CarlaSimulator
    : public SimulatorBase<CarlaSimulator<TranslationType>, TranslationType> {
 public:
  using BaseType =
      SimulatorBase<CarlaSimulator<TranslationType>, TranslationType>;
  using BaseType::BaseType;
  using SensorIDType = typename BaseType::SensorIDType;
  using FrameType = typename BaseType::FrameType;
  using LockType = typename BaseType::LockType;

  void InitImpl() {}

  void ConnectImpl() {
    logging::LogInfo(
        "CarlaViz is trying to connect to Carla simulator on {}:{}",
        this->option_.host.c_str(), this->option_.port);
    client_ = std::make_unique<carla::client::Client>(this->option_.host,
                                                      this->option_.port, 1);
    client_->SetTimeout(std::chrono::seconds(this->option_.timeout_seconds));
    logging::LogInfo(
        "CarlaViz connects to Carla simulator, Client API version: {}, Server "
        "API version: {}",
        client_->GetClientVersion(), client_->GetServerVersion());
  }

  void DisconnectImpl() {
    {
      std::lock_guard<LockType> guard(this->sensor_data_lock_);
      sensor_objects_.clear();
    }
    client_.reset(nullptr);
  }

  void ProcessImpl() {
    auto world = client_->GetWorld();
    CheckAndLoadMap(world);
    ProcessFrame(world);
    this->ProcessSensorData();
  }

  void RemoveSensorImpl(SensorIDType sensor_id) {
    // assume this function is called with mutex enabled
    auto sensor_itr = sensor_objects_.find(sensor_id);
    if (sensor_itr != sensor_objects_.end()) {
      sensor_itr->second->Stop();
      sensor_objects_.erase(sensor_itr);
      logging::LogInfo("Stop subscribing to sensor, id={}", sensor_id);
    } else {
      logging::LogError(
          "Stop subscribing to a sensor that has not been listened, either "
          "this sensor is not supported or something "
          "must be wrong, id={}",
          sensor_id);
    }

    lidar_points_.erase(sensor_id);
  }

 private:
  friend class SimulatorBase<CarlaSimulator<TranslationType>, TranslationType>;
  std::unique_ptr<carla::client::Client> client_{nullptr};
  std::unordered_map<SensorIDType, boost::shared_ptr<carla::client::Sensor>>
      sensor_objects_;
  // variables to remove stale point cloud
  std::unordered_map<SensorIDType, std::deque<sensor::LidarPointsWithTimestamp>>
      lidar_points_;

  void CheckAndLoadMap(const carla::client::World& world) {
    auto carla_map = world.GetMap();
    if (carla_map->GetName() != this->map_.Name()) {
      // first reset previous map
      this->map_.Reset();
      // set scale
      this->map_.SetScale({1, -1, 1});
      // load map
      logging::LogInfo("Loading map {} from Carla...", carla_map->GetName());
      this->map_.FromCarlaMap(*carla_map);
      AddEnvironmentObjects(world, *carla_map);
      this->translation_.UpdateMap(this->map_);
      logging::LogInfo("Loaded map {}.", this->map_.Name());
    }
  }

  void ProcessFrame(const carla::client::World& world) {
    auto world_snapshot =
        world.WaitForTick(std::chrono::seconds(this->option_.timeout_seconds));
    this->current_frame_.store(
        static_cast<FrameType>(world_snapshot.GetFrame()));

    auto now = world_snapshot.GetTimestamp().elapsed_seconds;

    this->translation_.UpdateTime(now, world_snapshot.GetFrame());

    // add objects/sensors
    for (const auto& actor_snapshot : world_snapshot) {
      auto actor = world.GetActor(actor_snapshot.id);
      logging::LogDebug("actor type: {}", actor->GetTypeId());
      if (utils::StartWith(actor->GetTypeId(), "vehicle")) {
        // handle vehicles
        if (IsEgoVehicle(*actor)) {
          this->translation_.UpdatePose(
              now, GetPosition(*actor), GetOrientation(*actor),
              actor->GetVelocity().Length(), actor->GetAcceleration().Length());
        } else {
          this->translation_.UpdateVehicle(actor->GetTypeId(),
                                           GetVerticesOfObject(*actor),
                                           GetHeightOfObject(*actor));
        }
      } else if (utils::StartWith(actor->GetTypeId(), "walker")) {
        // handle pedestrians
        this->translation_.UpdatePeople(actor->GetTypeId(),
                                        GetVerticesOfObject(*actor),
                                        GetHeightOfObject(*actor));
      } else if (actor->GetTypeId() == "traffic.traffic_light") {
        auto traffic_light =
            boost::dynamic_pointer_cast<carla::client::TrafficLight>(actor);
        if (!traffic_light) {
          logging::LogError(
              "Game actor of id {} has type id of traffic light but cannot be "
              "downcast to TrafficLight object, something must be wrong!",
              actor->GetId());
          continue;
        }
        this->translation_.UpdateTrafficLight(
            actor->GetId(),
            FromCarlaTrafficLightState(traffic_light->GetState()));
      } else if (utils::StartWith(actor->GetTypeId(), "sensor")) {
        // handle sensors
        if (!this->AddSensor(actor->GetId(), actor->GetTypeId())) {
          // we may already add this sensor in last loop
          // or this sensor is not enabled
          continue;
        }
        if (utils::StartWith(actor->GetTypeId(), "sensor.camera")) {
          ListenToSensor(
              actor,
              [this, actor](boost::shared_ptr<carla::sensor::SensorData> data) {
                this->HandleImageData(actor->GetId(), actor->GetTypeId(), data);
              });
        } else if (utils::StartWith(actor->GetTypeId(),
                                    "sensor.lidar.ray_cast")) {
          double rotation_frequency = 10.0;
          for (const auto& attribute : actor->GetAttributes()) {
            if (attribute.GetId() == "rotation_frequency") {
              rotation_frequency = std::stod(attribute.GetValue());
              break;
            }
          }
          if (actor->GetTypeId() == "sensor.lidar.ray_cast") {
            ListenToSensor(
                actor, [this, actor, rotation_frequency](
                           boost::shared_ptr<carla::sensor::SensorData> data) {
                  this->template HandleLidarMeasurementData<false>(
                      actor->GetId(), actor->GetTypeId(), data,
                      rotation_frequency);
                });
          } else {
            ListenToSensor(
                actor, [this, actor, rotation_frequency](
                           boost::shared_ptr<carla::sensor::SensorData> data) {
                  this->template HandleLidarMeasurementData<true>(
                      actor->GetId(), actor->GetTypeId(), data,
                      rotation_frequency);
                });
          }
        }
      }
    }

    this->ClearRemovedSensors();
  }

  bool IsEgoVehicle(const carla::client::Actor& actor) {
    for (const auto& attribute : actor.GetAttributes()) {
      if (attribute.GetId() == "role_name" &&
          (attribute.GetValue() == this->option_.ego_vehicle_name)) {
        return true;
      }
    }
    return false;
  }

  template <typename FuncType>
  void ListenToSensor(boost::shared_ptr<carla::client::Actor> actor,
                      FuncType&& func) {
    // Note: only process below sensors:
    //   cameras:
    //     sensor.camera.rgb
    //     sensor.camera.depth
    //     sensor.camera.semantic_segmentation
    //     sensor.camera.instance_segmentation
    //   lidars:
    //     sensor.lidar.ray_cast
    //     sensor.lidar.ray_cast_semantic
    auto sensor = boost::static_pointer_cast<carla::client::Sensor>(actor);
    auto id = static_cast<SensorIDType>(actor->GetId());
    auto type_id = actor->GetTypeId();

    logging::LogInfo("Subscribe to sensor, id={}, type={}", id, type_id);
    sensor->Listen(std::forward<FuncType>(func));
    std::lock_guard<LockType> guard(this->sensor_data_lock_);
    sensor_objects_[id] = sensor;
  }

  bool CheckSensorDataFrame(SensorIDType sensor_id, const std::string& type_id,
                            carla::SharedPtr<carla::sensor::SensorData> data) {
    auto sensor_data_frame = static_cast<FrameType>(data->GetFrame());
    auto current_frame = this->current_frame_.load();
    if (current_frame > sensor_data_frame &&
        (current_frame - sensor_data_frame) >
            this->option_.sensor_max_lag_frame) {
      logging::LogWarn(
          "Discarding data older than {}+ frames ago from sensor {} {}, "
          "current diff is {}. Creating dummy sensor data for it",
          this->option_.sensor_max_lag_frame, sensor_id, type_id,
          (current_frame - sensor_data_frame));
      return false;
    }
    return true;
  }

  template <bool IsSemantic>
  void HandleLidarMeasurementData(
      SensorIDType sensor_id, const std::string& type_id,
      carla::SharedPtr<carla::sensor::SensorData> data,
      double rotation_frequency) {
    if (!CheckSensorDataFrame(sensor_id, type_id, data)) {
      return;
    }
    using DataType =
        std::conditional_t<IsSemantic,
                           carla::sensor::data::SemanticLidarMeasurement,
                           carla::sensor::data::LidarMeasurement>;
    auto lidar_data = boost::dynamic_pointer_cast<DataType>(data);
    if (!lidar_data) {
      logging::LogError(
          "Cannot parse lidar data from sensor {} of id {}, something must be "
          "wrong!",
          type_id, sensor_id);
      return;
    }
    auto dot_pos = type_id.rfind(".");
    std::string lidar_type = "unknown";
    if (dot_pos != std::string::npos) {
      lidar_type = type_id.substr(dot_pos + 1);
    }
    sensor::LidarMeasurement out_lidar_measurement{
        .info = {.id = sensor_id,
                 .type = sensor::SensorType::LIDAR,
                 .name = lidar_type}};

    std::lock_guard<LockType> guard(this->sensor_data_lock_);
    auto lidar_points_itr = lidar_points_.find(sensor_id);
    if (lidar_points_itr == lidar_points_.end()) {
      lidar_points_itr =
          lidar_points_
              .insert(
                  {sensor_id, std::deque<sensor::LidarPointsWithTimestamp>()})
              .first;
    }

    while (!lidar_points_itr->second.empty() &&
           lidar_data->GetTimestamp() -
                   lidar_points_itr->second.front().timestamp >
               1.0 / rotation_frequency) {
      lidar_points_itr->second.pop_front();
    }

    lidar_points_itr->second.push_back(sensor::LidarPointsWithTimestamp{
        .timestamp = lidar_data->GetTimestamp(),
    });
    auto& new_points = lidar_points_itr->second.back();
    new_points.points.reserve(lidar_data->size() * 3);
    new_points.colors.reserve(lidar_data->size() * 4);

    auto sensor_transform = lidar_data->GetSensorTransform();
    for (const auto& point : *lidar_data) {
      auto p = point.point;
      sensor_transform.TransformPoint(p);
      new_points.points.push_back(p.x);
      new_points.points.push_back(-p.y);
      new_points.points.push_back(p.z);
      if constexpr (IsSemantic) {
        auto color_array =
            carla::image::CityScapesPalette::GetColor(point.object_tag);
        new_points.colors.push_back(color_array[0]);
        new_points.colors.push_back(color_array[1]);
        new_points.colors.push_back(color_array[2]);
        new_points.colors.push_back(255);  // alpha
      } else {
        new_points.colors.push_back(255);
        new_points.colors.push_back(255);
        new_points.colors.push_back(255);
        new_points.colors.push_back(255);
      }
    }

    for (const auto& points : lidar_points_itr->second) {
      out_lidar_measurement.points.insert(out_lidar_measurement.points.end(),
                                          points.points.begin(),
                                          points.points.end());
      out_lidar_measurement.colors.insert(out_lidar_measurement.colors.end(),
                                          points.colors.begin(),
                                          points.colors.end());
    }

    auto sensor_itr = this->sensor_status_.find(sensor_id);
    if (sensor_itr != this->sensor_status_.end() && sensor_itr->second) {
      this->sensor_data_[sensor_id] = std::move(out_lidar_measurement);
    }
  }

  void HandleImageData(SensorIDType sensor_id, const std::string& type_id,
                       carla::SharedPtr<carla::sensor::SensorData> data) {
    bool create_dummy = !CheckSensorDataFrame(sensor_id, type_id, data);
    auto image = boost::dynamic_pointer_cast<carla::sensor::data::Image>(data);
    if (!image) {
      logging::LogError("Cannot parse image data from sensor {} of id {}",
                        type_id, sensor_id);
      return;
    }
    assert(image->size() == image->GetWidth() * image->GetHeight());

    auto dot_pos = type_id.rfind(".");
    std::string camera_type = "unknown";
    if (dot_pos != std::string::npos) {
      camera_type = type_id.substr(dot_pos + 1);
    }

    sensor::Image out_image{.info = {.id = sensor_id,
                                     .type = sensor::SensorType::CAMERA,
                                     .name = camera_type}};

    if (create_dummy) {
      {
        std::lock_guard<LockType> guard(this->sensor_data_lock_);
        if (this->sensor_data_.find(sensor_id) != this->sensor_data_.end()) {
          // use previous image as dummy image
          return;
        }
      }
      std::vector<uint8_t> dummy_image_data = {0, 0, 0, 0};
      sensor::ConvertRawDataToImage<sensor::ImageColorType::BGRA,
                                    sensor::ImageColorType::RGB>(
          out_image, dummy_image_data.data(), 1, 1);
    } else {
      if (type_id == "sensor.camera.rgb") {
        sensor::ConvertRawDataToImage<sensor::ImageColorType::BGRA,
                                      sensor::ImageColorType::RGB>(
            out_image, reinterpret_cast<const uint8_t*>(image->data()),
            image->GetWidth(), image->GetHeight());
      } else if (type_id == "sensor.camera.depth") {
        std::vector<uint8_t> depth_data(image->size());
        for (std::size_t i = 0; i < image->size(); i++) {
          auto& p = (*image)[i];
          depth_data[i] = (uint8_t)((p.r + p.g * 256.0 + p.b * 256.0 * 256.0) /
                                    (256.0 * 256.0 * 256.0 - 1.0) * 255.0);
        }
        sensor::ConvertRawDataToImage<sensor::ImageColorType::GREY,
                                      sensor::ImageColorType::GREY>(
            out_image, depth_data.data(), image->GetWidth(),
            image->GetHeight());
      } else if (type_id == "sensor.camera.semantic_segmentation" ||
                 type_id == "sensor.camera.instance_segmentation") {
        std::vector<uint8_t> data(image->size() * 3);
        for (std::size_t i = 0; i < image->size(); i++) {
          auto& p = (*image)[i];
          auto color_array = carla::image::CityScapesPalette::GetColor(p.r);
          data[i * 3 + 0u] = color_array[0];
          data[i * 3 + 1u] = color_array[1];
          data[i * 3 + 2u] = color_array[2];
        }
        sensor::ConvertRawDataToImage<sensor::ImageColorType::RGB,
                                      sensor::ImageColorType::RGB>(
            out_image, reinterpret_cast<const uint8_t*>(data.data()),
            image->GetWidth(), image->GetHeight());
      } else {
        logging::LogError("Unknown camera type {}", type_id);
        return;
      }
    }
    std::lock_guard<LockType> guard(this->sensor_data_lock_);
    auto sensor_itr = this->sensor_status_.find(sensor_id);
    if (sensor_itr != this->sensor_status_.end() && sensor_itr->second) {
      this->sensor_data_[sensor_id] = std::move(out_image);
    }
  }

  float GetHeightOfObject(const carla::client::Actor& actor) {
    return actor.GetBoundingBox().extent.z * 2;
  }

  const std::array<float, 3> GetPosition(const carla::client::Actor& actor) {
    return {actor.GetLocation().x, -actor.GetLocation().y,
            actor.GetLocation().z};
  }

  const std::array<float, 3> GetOrientation(const carla::client::Actor& actor) {
    return {carla::geom::Math::ToRadians(actor.GetTransform().rotation.roll),
            -carla::geom::Math::ToRadians(actor.GetTransform().rotation.pitch),
            -carla::geom::Math::ToRadians(actor.GetTransform().rotation.yaw)};
  }

  std::vector<std::array<float, 3>> GetVerticesOfObject(
      const carla::client::Actor& actor) {
    std::vector<std::array<float, 3>> vertices =
        GetVerticesFromBoundingBoxAndTransform(actor.GetBoundingBox(),
                                               actor.GetTransform());

    logging::LogDebug("Added object {} of type {} with {} points, height: {}",
                      actor.GetId(), actor.GetTypeId(), vertices.size(),
                      actor.GetBoundingBox().extent.z);

    return vertices;
  }

  void AddEnvironmentObjects(const carla::client::World& world,
                             const carla::client::Map& map) {
    auto actor_snapshots =
        world.WaitForTick(std::chrono::seconds(this->option_.timeout_seconds));
    for (const auto& actor_snapshot : actor_snapshots) {
      auto actor = world.GetActor(actor_snapshot.id);
      if (!actor) {
        continue;
      }
      if (actor->GetTypeId() == "traffic.stop") {
        AddStopSign(actor, map);
      } else if (actor->GetTypeId() == "traffic.traffic_light") {
        AddTrafficLight(actor);
      }
    }

    for (const auto& env_obj : world.GetEnvironmentObjects(
             static_cast<uint8_t>(carla::rpc::CityObjectLabel::Any))) {
      switch (env_obj.type) {
        case carla::rpc::CityObjectLabel::Buildings:
          AddEnvironmentObject<map::Building>(env_obj);
          break;
        // case carla::rpc::CityObjectLabel::Sidewalks:
        //   AddEnvironmentObject<map::Sidewalk>(env_obj);
        //   break;
        case carla::rpc::CityObjectLabel::Poles:
          if (env_obj.name.find("TrafficLight") != std::string::npos) {
            AddEnvironmentObject<map::Pole>(env_obj, 0.3);
          }
          break;
        default:
          // not processing other types
          break;
      }
    }
  }

  void AddStopSign(boost::shared_ptr<carla::client::Actor> stop_sign,
                   const carla::client::Map& map) {
    auto stop_sign_waypoint = map.GetWaypoint(stop_sign->GetLocation(), false);
    auto stop_sign_transform = stop_sign->GetTransform();

    bool is_on_road = stop_sign_waypoint != nullptr;

    double scale = 1.0;

    if (is_on_road) {
      stop_sign_waypoint = map.GetWaypoint(stop_sign->GetLocation());
      stop_sign_transform = stop_sign_waypoint->GetTransform();
      stop_sign_transform.rotation.yaw += 90.0;
      scale = map::StopSign::stop_sign_length * 2 /
              stop_sign_waypoint->GetLaneWidth();
    }

    auto& stop_sign_in_map =
        this->map_.AddStopSign(stop_sign->GetId(), is_on_road, scale);

    // location
    stop_sign_in_map.transform.location.x = stop_sign_transform.location.x;
    stop_sign_in_map.transform.location.y = stop_sign_transform.location.y;
    stop_sign_in_map.transform.location.z = stop_sign_transform.location.z;

    // rotation
    stop_sign_in_map.transform.rotation.yaw =
        carla::geom::Math::ToRadians(stop_sign_transform.rotation.yaw);
    stop_sign_in_map.transform.rotation.pitch =
        carla::geom::Math::ToRadians(stop_sign_transform.rotation.pitch);
    stop_sign_in_map.transform.rotation.roll =
        carla::geom::Math::ToRadians(stop_sign_transform.rotation.roll);

    logging::LogDebug(
        "Added one stop sign {} at ({}, {}, {}) "
        "yaw {}",
        stop_sign->GetId(), stop_sign_in_map.transform.location.x,
        stop_sign_in_map.transform.location.y,
        stop_sign_in_map.transform.location.z,
        stop_sign_transform.rotation.yaw);
  }

  void AddTrafficLight(boost::shared_ptr<carla::client::Actor> actor) {
    auto traffic_light =
        boost::dynamic_pointer_cast<carla::client::TrafficLight>(actor);
    if (!traffic_light) {
      logging::LogError(
          "Game actor of id {} has type id of traffic light but cannot be "
          "downcast to TrafficLight object, something must be wrong!",
          actor->GetId());
      return;
    }
    map::TrafficLight& new_traffic_light =
        this->map_.AddTrafficLight(actor->GetId());

    // add traffic lights' trigger volume
    auto trigger_bbx = traffic_light->GetTriggerVolume();
    auto trigger_volume_vertices =
        trigger_bbx.GetWorldVertices(traffic_light->GetTransform());
    new_traffic_light.affected_regions.emplace_back(trigger_volume_vertices[0]);
    new_traffic_light.affected_regions.emplace_back(trigger_volume_vertices[2]);
    new_traffic_light.affected_regions.emplace_back(trigger_volume_vertices[6]);
    new_traffic_light.affected_regions.emplace_back(trigger_volume_vertices[4]);
    for (auto& point : new_traffic_light.affected_regions) {
      point.ApplyScale(this->map_.Scale());
    }

    // add actual traffic lights
    for (const auto& bbx : traffic_light->GetLightBoxes()) {
      auto world_vertices = bbx.GetLocalVertices();
      auto height = bbx.extent.z * 2;
      new_traffic_light.lights.emplace_back(height);
      new_traffic_light.lights.back().vertices.emplace_back(world_vertices[0]);
      new_traffic_light.lights.back().vertices.emplace_back(world_vertices[2]);
      new_traffic_light.lights.back().vertices.emplace_back(world_vertices[6]);
      new_traffic_light.lights.back().vertices.emplace_back(world_vertices[4]);
      for (auto& point : new_traffic_light.lights.back().vertices) {
        point.ApplyScale(this->map_.Scale());
      }
    }
  }

  template <typename EnvironmentObjectType>
  void AddEnvironmentObject(
      const carla::rpc::EnvironmentObject& env_obj,
      float constraint = std::numeric_limits<float>::max()) {
    auto& env_obj_in_map =
        this->map_.template AddStaticObject<EnvironmentObjectType>(env_obj.id);
    auto bbx_copy = env_obj.bounding_box;
    if constexpr (std::is_same_v<EnvironmentObjectType, map::Pole>) {
      ConstrainPoleTwoDimensions(bbx_copy.extent.x, bbx_copy.extent.y,
                                 bbx_copy.extent.z, constraint);
    }
    auto raw_vertices = bbx_copy.GetLocalVertices();
    env_obj_in_map.vertices.emplace_back(raw_vertices[0]);
    env_obj_in_map.vertices.emplace_back(raw_vertices[2]);
    env_obj_in_map.vertices.emplace_back(raw_vertices[6]);
    env_obj_in_map.vertices.emplace_back(raw_vertices[4]);
    env_obj_in_map.height = bbx_copy.extent.z * 2;
    logging::LogDebug("Added one static {} object id {}, name {}",
                      env_obj_in_map.type_str, env_obj_in_map.id, env_obj.name);
  }

  std::vector<std::array<float, 3>> GetVerticesFromBoundingBoxAndTransform(
      const carla::geom::BoundingBox& bounding_box,
      const carla::geom::Transform& transform, bool revert_y = true) {
    std::vector<std::array<float, 3>> vertices;
    double x_off = bounding_box.extent.x;
    double y_off = bounding_box.extent.y;
    carla::geom::Vector3D bounding_box_pos_0(-x_off, -y_off, 0);
    carla::geom::Vector3D bounding_box_pos_1(-x_off, y_off, 0);
    carla::geom::Vector3D bounding_box_pos_2(x_off, y_off, 0);
    carla::geom::Vector3D bounding_box_pos_3(x_off, -y_off, 0);
    transform.TransformPoint(bounding_box_pos_0);
    transform.TransformPoint(bounding_box_pos_1);
    transform.TransformPoint(bounding_box_pos_2);
    transform.TransformPoint(bounding_box_pos_3);
    std::vector<carla::geom::Vector3D> offset = {
        bounding_box_pos_0, bounding_box_pos_1, bounding_box_pos_2,
        bounding_box_pos_3};
    for (int j = 0; j < offset.size(); j++) {
      vertices.push_back(
          {offset[j].x, (revert_y ? -1 : 1) * offset[j].y, offset[j].z});
    }
    return vertices;
  }

  map::TrafficLightStatus FromCarlaTrafficLightState(
      carla::rpc::TrafficLightState state) {
    switch (state) {
      case carla::rpc::TrafficLightState::Red:
        return map::TrafficLightStatus::RED;
      case carla::rpc::TrafficLightState::Yellow:
        return map::TrafficLightStatus::YELLOW;
      case carla::rpc::TrafficLightState::Green:
        return map::TrafficLightStatus::GREEN;
      default:
        return map::TrafficLightStatus::UNKNOWN;
    }
  }

  void ConstrainPoleTwoDimensions(float& x, float& y, float& z,
                                  float constraint) {
    if (z >= x && z >= y) {
      x = std::min(x, constraint);
      y = std::min(y, constraint);
    } else if (y >= x && y >= z) {
      x = std::min(x, constraint);
      z = std::min(z, constraint);
    } else {
      z = std::min(z, constraint);
      y = std::min(y, constraint);
    }
  }
};

}  // namespace carlaviz::simulators
