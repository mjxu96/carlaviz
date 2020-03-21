/*
 * File: utils.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 3:18:48 pm
 */

#ifndef CARLAVIZ_UTILS_H_
#define CARLAVIZ_UTILS_H_

#include "backend/utils/def.h"
#include "backend/utils/json.hpp"

#include "carla/client/Map.h"
#include "carla/client/Sensor.h"
#include "carla/client/Waypoint.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/sensor/SensorData.h"

#include <fstream>
#include <iostream>
#include <tuple>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/shared_ptr.hpp>

namespace carlaviz {
namespace utils {

class Utils {
 public:
  static point_3d_t GetOffsetAfterTransform(const point_3d_t& origin,
                                            double yaw);
  static bool IsStartWith(const std::string& origin,
                          const std::string& pattern);
  static bool IsWithin(const point_3d_t& point,
                       const std::vector<point_3d_t>& polygon);
  static double ComputeSpeed(const carla::geom::Vector3D& velo);
};

class PointCloud {
 public:
  PointCloud() = default;
  PointCloud(double timestamp, std::vector<double>&& points);
  double GetTimestamp() const;
  std::vector<double>& GetPoints();

 private:
  std::vector<double> points_;
  double timestamp_;
};

class Image {
 public:
  Image() = default;
  Image(std::string&& encoded_str, size_t width, size_t height, double timestamp);
  std::string& GetData();
  size_t GetWidth() const;
  size_t GetHeight() const;
  double GetTimestamp() const;

  std::string encoded_str_{};
 private:
  int width_ = 0;
  int height_ = 0;
  double timestamp_ = 0;
};

class CollisionEvent {
 public:
  CollisionEvent() = default;
  CollisionEvent(uint32_t self_actor_id, uint32_t other_actor_id, 
    const std::string& self_actor_name, const std::string& other_actor_name, 
    double hit_timestamp, size_t last_hit_frame);
  
  std::string GetSelfActorName() const;
  std::string GetOtherActorName() const;
  double GetLastHitTimestamp() const;
  size_t GetLastHitFrame() const;
  
 private:
  uint32_t self_actor_id_{0u};
  uint32_t other_actor_id_{0u};
  std::string self_actor_name_{"null"};
  std::string other_actor_name_{"no collision"};
  double last_hit_timestamp_{-1.0};
  size_t last_hit_frame_{0u};
};

class GNSSInfo {
 public:
  GNSSInfo() = default;
  GNSSInfo(const std::string& self_actor_name, double lat, double lon, 
    double alt, double ts);
  std::string GetSelfActorName() const;
  std::vector<double> GetPositions() const;
  double GetTimestamp() const;
 private:
  std::string self_actor_name_{};
  double latitude_{-1.0};
  double longitude_{-1.0};
  double altitude_{-1.0};
  double timestamp_{-1.0};
};

class ObstacleInfo {
 public:
  ObstacleInfo() = default;
  ObstacleInfo(const std::string& self_actor_name, const std::string& other_actor_name,
    double dis, double timestamp, size_t frame);
  std::string GetSelfActorName() const;
  std::string GetOtherActorName() const;
  double GetDistance() const;
  size_t GetFrame() const;
  double GetTimestamp() const;
 private:
  std::string self_actor_name_{};
  std::string other_actor_name_{};
  double distance_{};
  double timestamp_{};
  size_t frame_{0u};
};

struct IMUInfo {
  IMUInfo() = default;
  IMUInfo(const std::string& actor_name, const std::vector<double>& acce, double comp,
    const std::vector<double>& gyro);
  std::string self_actor_name;
  std::vector<double> accelerometer;
  double compass;
  std::vector<double> gyroscope;
};

struct RadarInfo {
  RadarInfo() = default;
  RadarInfo(std::vector<double>&& p, std::vector<uint8_t>&& c);
  std::vector<double> points;
  std::vector<uint8_t> colors;
};

class XodrGeojsonConverter {
 public:
  static std::string Convert(std::string xodr);
  static std::string GetGeoJsonFromCarlaMap(
      boost::shared_ptr<carla::client::Map> map_ptr);
  static boost::geometry::model::point<double, 3,
                                       boost::geometry::cs::cartesian>
  LateralShift(carla::geom::Transform transform, double shift);

  static std::vector<double> LateralShiftGetVector(carla::geom::Transform transform, double shift);

 private:
  static nlohmann::json InitGeoJson();
  static void AddOneLine(
      const std::vector<boost::geometry::model::point<
          double, 3, boost::geometry::cs::cartesian>>& points,
      const uint32_t& road_id, nlohmann::json& json, const uint32_t& index);
  static void AddOneSide(
      const carla::SharedPtr<carla::client::Waypoint>& waypoint,
      nlohmann::json& json, const uint32_t& index);

  constexpr static const double precision_{0.5};
};

}  // namespace utils
}  // namespace carlaviz

#endif