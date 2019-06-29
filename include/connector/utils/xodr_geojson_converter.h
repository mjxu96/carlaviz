#ifndef ROTHEBERG_XODR_GEOJSON_CONVERTER_H_
#define ROTHEBERG_XODR_GEOJSON_CONVERTER_H_

#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/client/Waypoint.h"
#include "carla/client/Map.h"

#include "connector/utils/json.hpp"

#include <boost/geometry.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <fstream>

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;

namespace rothberg {
namespace utils {

class XodrGeojsonConverter {
public:
  static std::string Convert(std::string xodr);
  static std::string GetGeoJsonFromCarlaMap(boost::shared_ptr<carla::client::Map> map_ptr);

private:

  static nlohmann::json InitGeoJson();
  static void AddOneLine(const std::vector<point_t>& points, const uint32_t& road_id,
    nlohmann::json& json, const uint32_t& index);
  static void AddOneSide(const carla::SharedPtr<carla::client::Waypoint>& waypoint,
    nlohmann::json& json, const uint32_t& index);
    //const std::vector<point_t>& points, const uint32_t& road_id,
    //nlohmann::json& json, uint32_t& index);

  static point_t LateralShift(carla::geom::Transform transform, double shift);

  constexpr static const double precision_{0.05};


};

} // namespace utils
} // namespace rothberg


#endif