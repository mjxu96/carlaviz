/*
 * File: utils.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 3:18:48 pm
 */

#ifndef MELLOCOLATE_UTILS_H_
#define MELLOCOLATE_UTILS_H_

#include "platform/utils/def.h"
#include "platform/utils/json.hpp"

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

namespace mellocolate {
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
  PointCloud(double timestamp, std::vector<point_3d_t> points);
  double GetTimestamp() const;
  std::vector<point_3d_t> GetPoints() const;

 private:
  std::vector<point_3d_t> points_;
  double timestamp_;
};

class Image {
 public:
  Image() = default;
  Image(std::string encoded_str, size_t width, size_t height);
  std::string GetData() const;
  size_t GetWidth() const;
  size_t GetHeight() const;

 private:
  std::string encoded_str_{};
  int width_ = 0;
  int height_ = 0;
};

class XodrGeojsonConverter {
 public:
  static std::string Convert(std::string xodr);
  static std::string GetGeoJsonFromCarlaMap(
      boost::shared_ptr<carla::client::Map> map_ptr);
  static boost::geometry::model::point<double, 3,
                                       boost::geometry::cs::cartesian>
  LateralShift(carla::geom::Transform transform, double shift);

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
}  // namespace mellocolate

#endif