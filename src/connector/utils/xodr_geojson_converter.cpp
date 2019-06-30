
#include "connector/utils/xodr_geojson_converter.h"

using namespace rothberg::utils;

std::string XodrGeojsonConverter::Convert(std::string xodr) {
  //boost::optional<carla::road::Map> map = carla::opendrive::OpenDriveParser::Load(xodr);
  carla::client::Map map("map", xodr);
  /*
  if (map == boost::none) {
    std::cerr << "[XODR GEOJSON CONVERTER ERROR] Xodr string is not correct." << std::endl;
    return std::string();
  }
  auto topology = map.get().GenerateTopology();
  auto json = InitGeoJson();
  */
  /*
  for (const auto& point_pair : topology) {
    std::vector<point_t> points;
    points.push_back(LateralShift(point_pair.first))
  }
  */
  auto topology = map.GetTopology();
  auto json = InitGeoJson();
  uint32_t idx = 0u;
  for (const auto& point_pair : topology) {
    std::vector<point_t> points;
    points.push_back(LateralShift(point_pair.first->GetTransform(), point_pair.first->GetLaneWidth()));
    points.push_back(LateralShift(point_pair.second->GetTransform(), point_pair.second->GetLaneWidth()));
    AddOneLine(points, point_pair.first->GetRoadId(), json, idx);
    idx++;
  }
  return json.dump();
}

std::string XodrGeojsonConverter::GetGeoJsonFromCarlaMap(boost::shared_ptr<carla::client::Map> map_ptr) {
  auto topology = map_ptr->GetTopology();
  auto json = InitGeoJson();
  uint32_t idx = 0u;
  for (const auto& point_pair : topology) {
    //auto waypoint = point_pair.first;
    AddOneSide(point_pair.first, json, idx);
    AddOneSide(point_pair.second, json, idx+2);
    idx+=4;
  }
  return json.dump();
}

nlohmann::json XodrGeojsonConverter::InitGeoJson() {
  nlohmann::json json;
  json["type"] = "FeatureCollection";
  return std::move(json);
}

void XodrGeojsonConverter::AddOneSide(const carla::SharedPtr<carla::client::Waypoint>& waypoint,
  nlohmann::json& json, const uint32_t& index) {
    std::vector<carla::SharedPtr<carla::client::Waypoint>> tmp_waypoints;
    uint32_t road_id = waypoint->GetRoadId();
    auto next_waypoints = waypoint->GetNext(precision_);
    tmp_waypoints.push_back(waypoint);
    while (!next_waypoints.empty()) {
      auto next_waypoint = next_waypoints[0];
      if (next_waypoint->GetRoadId() == road_id) {
        tmp_waypoints.push_back(next_waypoint);
        next_waypoints = next_waypoint->GetNext(precision_);
      } else {
        break;
      }
    }
    std::vector<point_t> points;
    for (const auto& waypoint : tmp_waypoints) {
      points.push_back(LateralShift(waypoint->GetTransform(), -waypoint->GetLaneWidth() * 0.5));
    }
    AddOneLine(points, road_id, json, index);
    points.clear();
    for (const auto& waypoint : tmp_waypoints) {
      points.push_back(LateralShift(waypoint->GetTransform(), waypoint->GetLaneWidth() * 0.5));
    }
    AddOneLine(points, road_id, json, index+1);
    //points.push_back(LateralShift(point_pair.first->GetTransform(), point_pair.first->GetLaneWidth()));
    //points.push_back(LateralShift(point_pair.second->GetTransform(), point_pair.second->GetLaneWidth()));
}

void XodrGeojsonConverter::AddOneLine(const std::vector<point_t>& points, const uint32_t& road_id,
    nlohmann::json& json, const uint32_t& index) {
  json["features"][index]["type"] = "Feature";
  json["features"][index]["id"] = std::to_string(index);
  json["features"][index]["properties"]["name"] = std::to_string(road_id);
  json["features"][index]["geometry"]["type"] = "LineString";
  int i = 0;
  for (const auto& point : points) {
    json["features"][index]["geometry"]["coordinates"][i][0] = point.get<0>();
    json["features"][index]["geometry"]["coordinates"][i][1] = point.get<1>();
    json["features"][index]["geometry"]["coordinates"][i][2] = point.get<2>();
    i++;
  }
}

point_t XodrGeojsonConverter::LateralShift(carla::geom::Transform transform, double shift) {
  transform.rotation.yaw += 90.0;
  point_t p1(transform.location.x, transform.location.y, transform.location.z);
  auto p2_tmp = shift * transform.GetForwardVector();
  point_t p2(p2_tmp.x, p2_tmp.y, p2_tmp.z);
  //auto point = transform.location + shift * transform.GetForwardVector();
  return point_t(p1.get<0>() + p2.get<0>(), p1.get<1>() + p2.get<1>(), p1.get<2>() + p2.get<2>());
}

// Save following previous codes for future reference
/*
  auto& map_data = map.get().GetMap();
  auto& road_map = map_data.GetRoads();

  for (const auto& road_pair : road_map) {
    double length = road_pair.second.GetLength();
    std::cout << "road length: " << length << std::endl;
    double s = 0;
    double t = 0;
    void* pre_addr = 0x0;
    while (s < length) {

    const carla::road::element::RoadInfoGeometry* road_info_geo = road_pair.second.GetInfo<carla::road::element::RoadInfoGeometry>(s);

    auto road_geometry_ptr = &(road_info_geo->GetGeometry());//std::make_shared<carla::road::element::GeometryLine>(road_info_geo->GetGeometry());
    if (road_geometry_ptr != pre_addr) {
      t = 0;
      pre_addr = (void*)road_geometry_ptr;
    }
    std::cout << (int)(road_geometry_ptr->GetType()) << std::endl;
    std::cout << "pos: " << road_geometry_ptr->PosFromDist(t).location.x << ", " << road_geometry_ptr->PosFromDist(t).location.y << std::endl;
    t += 1;
    s += 1;
    }
    */
    /* 
    std::cout << "start point: " << road_geometry_ptr->GetStartPosition().x << ", " << road_geometry_ptr->GetStartPosition().y << std::endl;
    road_info_geo = road_pair.second.GetInfo<carla::road::element::RoadInfoGeometry>(length-15);
    std::cout << (int)(road_geometry_ptr->GetType()) << std::endl;
    //road_geometry = road_info_geo->GetGeometry();
    road_geometry_ptr = &(road_info_geo->GetGeometry());//std::make_shared<carla::road::element::GeometryLine>(road_info_geo->GetGeometry());
    std::cout << "start point: " << road_geometry_ptr->GetStartPosition().x << ", " << road_geometry_ptr->GetStartPosition().y << std::endl;
    break;
  }
    */


/*
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "indicate input file" << std::endl;
    return 0;
  }
  std::ifstream t(argv[1]);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string geojson_str = XodrGeojsonConverter::Convert(buffer.str());
  t.close();

  std::ofstream geojson_file("example.geojson", std::ios::out | std::ios::trunc);
  if (geojson_file.is_open()) {
    geojson_file << geojson_str;
    geojson_file.close();
  } else {
    std::cerr << "Not open" << std::endl;
  }
  //XodrGeojsonConverter::Convert(buffer.str());
  return 0;
}
*/