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

#ifdef CARLAVIZ_SIMULATOR_CARLA
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#endif

#ifdef CARLAVIZ_FRONTEND_XVIZ
#include <nlohmann/json.hpp>
#endif

#include "geometry.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace carlaviz::map {

namespace detail {
template <typename... StaticObjectTypes>
class StaticObjectSelector;

template <typename StaticObjectType>
class StaticObjectSelector<StaticObjectType> {
 public:
  template <typename TargetStaticObjectType>
  TargetStaticObjectType& Add(uint64_t id) requires(
      std::is_same_v<TargetStaticObjectType, StaticObjectType>) {
    objects_.emplace_back(TargetStaticObjectType{.id = id});
    return objects_.back();
  }
  template <typename TargetStaticObjectType>
  const std::vector<TargetStaticObjectType>& GetObjects() const
      requires(std::is_same_v<TargetStaticObjectType, StaticObjectType>) {
    return objects_;
  }
  void Clear() { objects_.clear(); }

 private:
  std::vector<StaticObjectType> objects_;
};

template <typename StaticObjectType, typename... StaticObjectTypes>
class StaticObjectSelector<StaticObjectType, StaticObjectTypes...> {
 public:
  template <typename TargetStaticObjectType>
  TargetStaticObjectType& Add(uint64_t id) {
    if constexpr (std::is_same_v<TargetStaticObjectType, StaticObjectType>) {
      objects_.emplace_back(TargetStaticObjectType{.id = id});
      return objects_.back();
    } else {
      return next_.template Add<TargetStaticObjectType>(id);
    }
  }

  template <typename TargetStaticObjectType>
  const std::vector<TargetStaticObjectType>& GetObjects() const {
    if constexpr (std::is_same_v<TargetStaticObjectType, StaticObjectType>) {
      return objects_;
    } else {
      return next_.template GetObjects<TargetStaticObjectType>();
    }
  }

  void Clear() {
    objects_.clear();
    next_.Clear();
  }

 private:
  StaticObjectSelector<StaticObjectTypes...> next_;
  std::vector<StaticObjectType> objects_;
};

}  // namespace detail

using Point = utils::Location;

struct Line {
  std::vector<Point> points;
};

struct Road {
  uint64_t road_id = 0;
  Line left_boundary;
  Line right_boundary;
};

struct StopSign {
  uint64_t stop_sign_id = 0;
  utils::Transform transform;

  constexpr static double stop_sign_height = 2.0;
  constexpr static double stop_sign_length = 1.4;
  constexpr static double stop_sign_stroke_length = 0.5;

  const static std::vector<std::vector<Point>> boarder;
  const static std::vector<std::vector<Point>> word;

  bool is_on_road = false;
  double scale = 1.0;
  const static utils::Transform on_road_transform;

  std::vector<std::vector<Point>> GetTransformedBoarder() const {
    std::vector<std::vector<Point>> ret;
    for (const auto& part : boarder) {
      std::vector<Point> tp;
      for (const auto& bp : part) {
        auto new_bp = bp * scale;
        if (is_on_road) {
          new_bp.z -= stop_sign_height * scale;
          new_bp.y = 0;
          on_road_transform.TransformLocation(new_bp);
        }
        transform.TransformLocation(new_bp);
        tp.emplace_back(new_bp);
      }
      ret.push_back(tp);
      // on road stop sign on need part 1
      if (is_on_road) {
        break;
      }
    }
    return ret;
  }

  std::vector<std::vector<Point>> GetTransformedWord() const {
    std::vector<std::vector<Point>> ret;
    for (const auto& character : word) {
      std::vector<Point> tc;
      for (const auto& stroke : character) {
        auto new_stroke = stroke * scale;
        if (is_on_road) {
          new_stroke.z -= stop_sign_height * scale;
          new_stroke.y = 0;
          on_road_transform.TransformLocation(new_stroke);
        }
        transform.TransformLocation(new_stroke);
        tc.emplace_back(new_stroke);
      }
      ret.push_back(tc);
    }
    return ret;
  }
};

struct Building {
  uint64_t id = 0;
  std::vector<Point> vertices;
  double height = 0;
  constexpr static std::string_view type_str = "building";
};

struct Sidewalk {
  uint64_t id = 0;
  std::vector<Point> vertices;
  double height = 0;
  constexpr static std::string_view type_str = "sidewalk";
};

struct Plant {
  uint64_t id = 0;
  std::vector<Point> vertices;
  double height = 0;
  constexpr static std::string_view type_str = "plant";
};

struct Pole {
  uint64_t id = 0;
  std::vector<Point> vertices;
  double height = 0;
  constexpr static std::string_view type_str = "pole";
};

enum class TrafficLightStatus { RED = 0, YELLOW, GREEN, UNKNOWN };

std::string TrafficLightStatusToString(TrafficLightStatus status);

struct TrafficLight {
  struct Light {
    Light(float h) : height(h) {}
    std::vector<Point> vertices;
    float height = 0.0f;
  };
  uint64_t id = 0;
  std::vector<Light> lights;

  std::vector<Point> affected_regions;
};

std::vector<std::array<float, 3>> PointsVectorToArrayVector(
    const std::vector<Point>& input);
std::vector<Point> ArrayVectorToPointsVector(
    const std::vector<std::array<float, 3>>& input);

class Map {
 public:
  Map(const std::string& name) : name_(name) {}

  Map() = default;

  void SetName(const std::string& name) { name_ = name; }
  std::string Name() const { return name_; }

  void SetScale(const std::array<double, 3>& scale) { scale_ = scale; }
  std::array<double, 3> Scale() { return scale_; }

  Road& AddRoad(uint64_t road_id) {
    roads_.emplace_back(Road{.road_id = road_id});
    return roads_.back();
  }

  StopSign& AddStopSign(uint64_t stop_sign_id, bool is_on_road, double scale) {
    stop_signs_.emplace_back(StopSign{.stop_sign_id = stop_sign_id,
                                      .is_on_road = is_on_road,
                                      .scale = scale});
    return stop_signs_.back();
  }

  TrafficLight& AddTrafficLight(uint64_t id) {
    traffic_lights_.emplace_back(TrafficLight{.id = id});
    return traffic_lights_.back();
  }

  template <typename T>
  T& AddStaticObject(uint64_t id) {
    return static_objects_.template Add<T>(id);
  }

  template <typename T>
  const std::vector<T>& GetStaticObjects() const {
    return static_objects_.template GetObjects<T>();
  }

  const std::vector<Road>& Roads() const { return roads_; }

  const std::vector<TrafficLight>& TrafficLights() const {
    return traffic_lights_;
  }

  void Reset() {
    name_.clear();
    scale_ = {1, 1, 1};
    roads_.clear();
    stop_signs_.clear();
    traffic_lights_.clear();
    static_objects_.Clear();
  }

  bool Empty() {
    return name_.empty() && roads_.empty() && stop_signs_.empty() &&
           traffic_lights_.empty();
  }

#ifdef CARLAVIZ_SIMULATOR_CARLA
 public:
  void FromCarlaMap(const carla::client::Map& map) {
    name_ = map.GetName();
    for (const auto& [point_start, point_end] : map.GetTopology()) {
      AddOneRoad(point_start);
      AddOneRoad(point_end);
    }
  }

 private:
  void AddOneRoad(const carla::SharedPtr<carla::client::Waypoint>& waypoint) {
    std::vector<carla::SharedPtr<carla::client::Waypoint>> tmp_waypoints;
    uint32_t road_id = waypoint->GetRoadId();
    double precision = 0.5;
    auto next_waypoints = waypoint->GetNext(precision);
    tmp_waypoints.push_back(waypoint);
    while (!next_waypoints.empty()) {
      auto next_waypoint = next_waypoints[0];
      if (next_waypoint->GetRoadId() == road_id) {
        tmp_waypoints.push_back(next_waypoint);
        next_waypoints = next_waypoint->GetNext(precision);
      } else {
        break;
      }
    }
    if (tmp_waypoints.empty()) {
      return;
    }
    Road& new_road = AddRoad(road_id);
    std::vector<Point> points;
    for (const auto& waypoint : tmp_waypoints) {
      new_road.left_boundary.points.push_back(LateralShift(
          waypoint->GetTransform(), -waypoint->GetLaneWidth() * 0.5));
      new_road.right_boundary.points.push_back(LateralShift(
          waypoint->GetTransform(), waypoint->GetLaneWidth() * 0.5));
    }
  }

  Point LateralShift(carla::geom::Transform transform, double shift) {
    transform.rotation.yaw += 90.0;
    auto p2_tmp = shift * transform.GetForwardVector();
    return {
        transform.location.x + p2_tmp.x,
        transform.location.y + p2_tmp.y,
        transform.location.z + p2_tmp.z,
    };
  }
#endif

#ifdef CARLAVIZ_FRONTEND_XVIZ
 public:
  nlohmann::json ToGeoJson(bool allow_static_objects) const {
    nlohmann::json json;
    json["type"] = "FeatureCollection";
    uint64_t idx = 0;
    for (const auto& road : roads_) {
      AddOneLineIntoGeoJson(road.left_boundary.points, "road", json, idx);
      AddOneLineIntoGeoJson(road.right_boundary.points, "road", json, idx + 1);
      idx += 2;
    }
    for (const auto& stop_sign : stop_signs_) {
      for (const auto& boarder : stop_sign.GetTransformedBoarder()) {
        AddOneLineIntoGeoJson(boarder, "stop_sign", json, idx);
        idx += 1;
      }
      auto transformed_word = stop_sign.GetTransformedWord();
      for (const auto& character : transformed_word) {
        AddOneLineIntoGeoJson(character, "stop_sign", json, idx);
        idx += 1;
      }
    }

    if (!allow_static_objects) {
      return json;
    }

    // below are static objects
    // add buildings
    for (const auto& obj : static_objects_.template GetObjects<Building>()) {
      AddPolygonWithHeight(obj.vertices, obj.height, obj.type_str, json, idx);
      idx += 1;
    }

    // add sidewalk
    for (const auto& obj : static_objects_.template GetObjects<Sidewalk>()) {
      AddPolygonWithHeight(obj.vertices, obj.height, obj.type_str, json, idx);
      idx += 1;
    }

    for (const auto& obj : static_objects_.template GetObjects<Pole>()) {
      AddPolygonWithHeight(obj.vertices, obj.height, obj.type_str, json, idx);
      idx += 1;
    }

    // add plant
    for (const auto& obj : static_objects_.template GetObjects<Plant>()) {
      AddPolygonWithHeight(obj.vertices, obj.height, obj.type_str, json, idx);
      idx += 1;
    }
    return json;
  }

 private:
  void AddOneLineIntoGeoJson(const std::vector<Point>& points,
                             const std::string& type, nlohmann::json& json,
                             uint64_t index) const {
    json["features"][index]["type"] = "Feature";
    json["features"][index]["id"] = std::to_string(index);
    json["features"][index]["properties"]["type"] = type;
    json["features"][index]["geometry"]["type"] = "LineString";
    int i = 0;
    for (const auto& point : points) {
      json["features"][index]["geometry"]["coordinates"][i][0] =
          point.x * scale_[0];
      json["features"][index]["geometry"]["coordinates"][i][1] =
          point.y * scale_[1];
      json["features"][index]["geometry"]["coordinates"][i][2] =
          point.z * scale_[2];
      i++;
    }
  }

  void AddPolygonWithHeight(const std::vector<Point>& vertices, double height,
                            std::string_view type, nlohmann::json& json,
                            uint64_t index) const {
    json["features"][index]["type"] = "Feature";
    json["features"][index]["id"] = std::to_string(index);
    json["features"][index]["properties"]["type"] = type;
    json["features"][index]["properties"]["height"] = height;
    json["features"][index]["geometry"]["type"] = "Polygon";
    int i = 0;
    for (const auto& point : vertices) {
      json["features"][index]["geometry"]["coordinates"][0][i][0] =
          point.x * scale_[0];
      json["features"][index]["geometry"]["coordinates"][0][i][1] =
          point.y * scale_[1];
      json["features"][index]["geometry"]["coordinates"][0][i][2] =
          point.z * scale_[2];
      i++;
    }
    // add the first point to close the polygon
    json["features"][index]["geometry"]["coordinates"][0][i][0] =
        vertices[0].x * scale_[0];
    json["features"][index]["geometry"]["coordinates"][0][i][1] =
        vertices[0].y * scale_[1];
    json["features"][index]["geometry"]["coordinates"][0][i][2] =
        vertices[0].z * scale_[2];
  }
#endif

 private:
  std::string name_;
  std::array<double, 3> scale_{1, 1, 1};
  std::vector<Road> roads_;
  std::vector<StopSign> stop_signs_;
  std::vector<TrafficLight> traffic_lights_;

  detail::StaticObjectSelector<Building, Sidewalk, Pole, Plant> static_objects_;
};

}  // namespace carlaviz::map
