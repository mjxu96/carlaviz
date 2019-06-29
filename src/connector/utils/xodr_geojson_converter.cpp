
#include "connector/utils/xodr_geojson_converter.h"

using namespace rothberg::utils;

std::string XodrGeojsonConverter::Convert(std::string xodr) {
  boost::optional<carla::road::Map> map = carla::opendrive::OpenDriveParser::Load(xodr);
  if (map == boost::none) {
    std::cerr << "[XODR GEOJSON CONVERTER ERROR] Xodr string is not correct." << std::endl;
    return std::string();
  }
  auto& map_data = map.get().GetMap();
  auto& road_map = map_data.GetRoads();

/*
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

  return std::string();
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "indicate input file" << std::endl;
    return 0;
  }
  std::ifstream t(argv[1]);
  std::stringstream buffer;
  buffer << t.rdbuf();
  XodrGeojsonConverter::Convert(buffer.str());
  return 0;
}