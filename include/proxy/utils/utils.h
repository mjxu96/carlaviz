/*
 * File: utils.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 3:18:48 pm
 */

#ifndef MELLOCOLATE_UTILS_H_
#define MELLOCOLATE_UTILS_H_

#include "proxy/utils/def.h"
#include "proxy/utils/json.hpp"

#include "carla/client/Map.h"
#include "carla/client/Waypoint.h"
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/sensor/SensorData.h"
#include "carla/client/Sensor.h"

#include <tuple>
#include <fstream>
#include <iostream>

#include <boost/geometry.hpp>
#include <boost/shared_ptr.hpp>


namespace mellocolate {
namespace utils {

/*
class SensorDataPackage {
public:
  SensorDataPackage() = delete;
  SensorDataPackage(carla::sensor::SensorData sensor_data, carla::geom::Location location,
    carla::geom::Transform transform) :
    sensor_data_(std::move(sensor_data)),
    location_(std::move(location)),
    transform_(std::move(transform)) {}

private:
  carla::sensor::SensorData sensor_data_;
  carla::geom::Location location_;
  carla::geom::Transform transform_;
};
*/
// rotate with yaw in radian
class Utils {
public:
  static point_3d_t GetOffsetAfterTransform(const point_3d_t& origin, double yaw);
  static bool IsStartWith(const std::string& origin, const std::string& pattern);
};

class XodrGeojsonConverter {
 public:
  static std::string Convert(std::string xodr);
  static std::string GetGeoJsonFromCarlaMap(
      boost::shared_ptr<carla::client::Map> map_ptr);

 private:
  static nlohmann::json InitGeoJson();
  static void AddOneLine(
      const std::vector<boost::geometry::model::point<
          double, 3, boost::geometry::cs::cartesian>>& points,
      const uint32_t& road_id, nlohmann::json& json, const uint32_t& index);
  static void AddOneSide(
      const carla::SharedPtr<carla::client::Waypoint>& waypoint,
      nlohmann::json& json, const uint32_t& index);

  static boost::geometry::model::point<double, 3,
                                       boost::geometry::cs::cartesian>
  LateralShift(carla::geom::Transform transform, double shift);

  constexpr static const double precision_{0.5};
};

}  // namespace utils
}  // namespace mellocolate

#endif