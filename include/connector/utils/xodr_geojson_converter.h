#ifndef MELLOCOLATE_XODR_GEOJSON_CONVERTER_H_
#define MELLOCOLATE_XODR_GEOJSON_CONVERTER_H_

#include "carla/client/Map.h"
#include "carla/client/Waypoint.h"
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoGeometry.h"

#include "connector/utils/json.hpp"

#include <boost/geometry.hpp>
#include <boost/shared_ptr.hpp>

#include <fstream>
#include <iostream>


namespace mellocolate {
namespace utils {

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