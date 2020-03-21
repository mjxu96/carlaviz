/*
 * File: utils.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:36:02 pm
 */

#include "backend/utils/utils.h"

using namespace carlaviz::utils;
using point_3d_t =
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

point_3d_t Utils::GetOffsetAfterTransform(const point_3d_t& origin,
                                          double yaw) {
  double x = origin.get<0>();
  double y = origin.get<1>();
  return point_3d_t(std::cos(yaw) * x - std::sin(yaw) * y,
                    std::sin(yaw) * x + std::cos(yaw) * y, origin.get<2>());
}

bool Utils::IsStartWith(const std::string& origin, const std::string& pattern) {
  size_t o_len = origin.size();
  size_t p_len = pattern.size();
  if (p_len <= 0u) {
    return true;
  }
  if (o_len < p_len) {
    return false;
  }
  return (origin.substr(0, p_len) == pattern);
}

bool Utils::IsWithin(const point_3d_t& point,
                     const std::vector<point_3d_t>& polygon) {
  typedef boost::geometry::model::d2::point_xy<double> point_type;
  typedef boost::geometry::model::polygon<point_type> polygon_type;
  point_type poi(point.get<0>(), point.get<1>());
  std::vector<point_type> points;
  for (const auto& point : polygon) {
    points.emplace_back(point.get<0>(), point.get<1>());
  }
  points.emplace_back(polygon[0].get<0>(), polygon[0].get<1>());
  polygon_type p;
  boost::geometry::assign_points(p, points);
  return boost::geometry::within(poi, p);
}

double Utils::ComputeSpeed(const carla::geom::Vector3D& velo) {
  double res = velo.x * velo.x + velo.y * velo.y + velo.z * velo.z;
  return std::sqrt(res);
}

PointCloud::PointCloud(double timestamp, std::vector<double>&& points)
    : timestamp_(timestamp), points_(std::move(points)) {}

double PointCloud::GetTimestamp() const { return timestamp_; }

std::vector<double>& PointCloud::GetPoints() { return points_; }

Image::Image(std::string&& encoded_str, size_t width, size_t height, double timestamp)
    : encoded_str_(std::move(encoded_str)), width_(width), height_(height), timestamp_(timestamp) {}

std::string& Image::GetData() { return encoded_str_; }
size_t Image::GetWidth() const { return width_; }
size_t Image::GetHeight() const { return height_; }
double Image::GetTimestamp() const { return timestamp_; }

CollisionEvent::CollisionEvent(uint32_t self_actor_id, uint32_t other_actor_id, 
    const std::string& self_actor_name, const std::string& other_actor_name, 
    double hit_timestamp, size_t hit_frame) :
    self_actor_id_(self_actor_id), other_actor_id_(other_actor_id), self_actor_name_(self_actor_name),
    other_actor_name_(other_actor_name), last_hit_timestamp_(hit_timestamp), last_hit_frame_(hit_frame) {}

std::string CollisionEvent::GetSelfActorName() const { return self_actor_name_; }
std::string CollisionEvent::GetOtherActorName() const { return other_actor_name_; }
double CollisionEvent::GetLastHitTimestamp() const { return last_hit_timestamp_; }
size_t CollisionEvent::GetLastHitFrame() const { return last_hit_frame_; }

GNSSInfo::GNSSInfo(const std::string& self_actor_name, double lat, double lon, double alt, double ts) :
  self_actor_name_(self_actor_name), latitude_(lat), longitude_(lon), altitude_(alt), timestamp_(ts) {}
std::string GNSSInfo::GetSelfActorName() const {
  return self_actor_name_;
}
std::vector<double> GNSSInfo::GetPositions() const {
  return {latitude_, longitude_, altitude_};
}
double GNSSInfo::GetTimestamp() const {
  return timestamp_;
}

ObstacleInfo::ObstacleInfo(const std::string& self_actor_name, const std::string& other_actor_name,
  double dis, double timestamp, size_t frame) : self_actor_name_(self_actor_name), other_actor_name_(other_actor_name), 
    distance_(dis), timestamp_(timestamp), frame_(frame) {}
std::string ObstacleInfo::GetSelfActorName() const {
  return self_actor_name_;
}

std::string ObstacleInfo::GetOtherActorName() const {
  return other_actor_name_;
}

double ObstacleInfo::GetDistance() const {
  return distance_;
}

size_t ObstacleInfo::GetFrame() const {
  return frame_;
}

double ObstacleInfo::GetTimestamp() const {
  return timestamp_;
}

IMUInfo::IMUInfo(const std::string& actor_name, const std::vector<double>& acce, double comp,
    const std::vector<double>& gyro) : self_actor_name(actor_name), accelerometer(acce), compass(comp),
      gyroscope(gyro) {}

RadarInfo::RadarInfo(std::vector<double>&& p, std::vector<uint8_t>&& c) :
  points(std::move(p)), colors(std::move(c)) {}

std::string XodrGeojsonConverter::Convert(std::string xodr) {
  carla::client::Map map("map", xodr);

  auto topology = map.GetTopology();
  auto json = InitGeoJson();
  uint32_t idx = 0u;
  for (const auto& point_pair : topology) {
    std::vector<point_3d_t> points;
    points.push_back(LateralShift(point_pair.first->GetTransform(),
                                  point_pair.first->GetLaneWidth()));
    points.push_back(LateralShift(point_pair.second->GetTransform(),
                                  point_pair.second->GetLaneWidth()));
    AddOneLine(points, point_pair.first->GetRoadId(), json, idx);
    idx++;
  }
  return json.dump();
}

std::string XodrGeojsonConverter::GetGeoJsonFromCarlaMap(
    boost::shared_ptr<carla::client::Map> map_ptr) {
  auto topology = map_ptr->GetTopology();
  auto json = InitGeoJson();
  uint32_t idx = 0u;
  for (const auto& point_pair : topology) {
    // auto waypoint = point_pair.first;
    AddOneSide(point_pair.first, json, idx);
    AddOneSide(point_pair.second, json, idx + 2);
    idx += 4;
  }
  return json.dump();
}

nlohmann::json XodrGeojsonConverter::InitGeoJson() {
  nlohmann::json json;
  json["type"] = "FeatureCollection";
  return std::move(json);
}

void XodrGeojsonConverter::AddOneSide(
    const carla::SharedPtr<carla::client::Waypoint>& waypoint,
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
  std::vector<point_3d_t> points;
  for (const auto& waypoint : tmp_waypoints) {
    points.push_back(LateralShift(waypoint->GetTransform(),
                                  -waypoint->GetLaneWidth() * 0.5));
  }
  AddOneLine(points, road_id, json, index);
  points.clear();
  for (const auto& waypoint : tmp_waypoints) {
    points.push_back(
        LateralShift(waypoint->GetTransform(), waypoint->GetLaneWidth() * 0.5));
  }
  AddOneLine(points, road_id, json, index + 1);
}

void XodrGeojsonConverter::AddOneLine(const std::vector<point_3d_t>& points,
                                      const uint32_t& road_id,
                                      nlohmann::json& json,
                                      const uint32_t& index) {
  json["features"][index]["type"] = "Feature";
  json["features"][index]["id"] = std::to_string(index);
  json["features"][index]["properties"]["name"] = std::to_string(road_id);
  json["features"][index]["geometry"]["type"] = "LineString";
  int i = 0;
  for (const auto& point : points) {
    json["features"][index]["geometry"]["coordinates"][i][0] = point.get<0>();
    json["features"][index]["geometry"]["coordinates"][i][1] = -point.get<1>();
    json["features"][index]["geometry"]["coordinates"][i][2] = point.get<2>();
    i++;
  }
}

point_3d_t XodrGeojsonConverter::LateralShift(carla::geom::Transform transform,
                                              double shift) {
  transform.rotation.yaw += 90.0;
  point_3d_t p1(transform.location.x, transform.location.y,
                transform.location.z);
  auto p2_tmp = shift * transform.GetForwardVector();
  point_3d_t p2(p2_tmp.x, p2_tmp.y, p2_tmp.z);
  // auto point = transform.location + shift * transform.GetForwardVector();
  return point_3d_t(p1.get<0>() + p2.get<0>(), p1.get<1>() + p2.get<1>(),
                    p1.get<2>() + p2.get<2>());
}

std::vector<double> XodrGeojsonConverter::LateralShiftGetVector(carla::geom::Transform transform, double shift) {
  transform.rotation.yaw += 90.0;
  // point_3d_t p1(transform.location.x, transform.location.y,
  //               transform.location.z);
  auto p2_tmp = shift * transform.GetForwardVector();
  return {transform.location.x + p2_tmp.x, transform.location.y + p2_tmp.y, transform.location.z + p2_tmp.z};
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

    const carla::road::element::RoadInfoGeometry* road_info_geo =
  road_pair.second.GetInfo<carla::road::element::RoadInfoGeometry>(s);

    auto road_geometry_ptr =
  &(road_info_geo->GetGeometry());//std::make_shared<carla::road::element::GeometryLine>(road_info_geo->GetGeometry());
    if (road_geometry_ptr != pre_addr) {
      t = 0;
      pre_addr = (void*)road_geometry_ptr;
    }
    std::cout << (int)(road_geometry_ptr->GetType()) << std::endl;
    std::cout << "pos: " << road_geometry_ptr->PosFromDist(t).location.x << ", "
  << road_geometry_ptr->PosFromDist(t).location.y << std::endl; t += 1; s += 1;
    }
    */
/*
std::cout << "start point: " << road_geometry_ptr->GetStartPosition().x << ", "
<< road_geometry_ptr->GetStartPosition().y << std::endl; road_info_geo =
road_pair.second.GetInfo<carla::road::element::RoadInfoGeometry>(length-15);
std::cout << (int)(road_geometry_ptr->GetType()) << std::endl;
//road_geometry = road_info_geo->GetGeometry();
road_geometry_ptr =
&(road_info_geo->GetGeometry());//std::make_shared<carla::road::element::GeometryLine>(road_info_geo->GetGeometry());
std::cout << "start point: " << road_geometry_ptr->GetStartPosition().x << ", "
<< road_geometry_ptr->GetStartPosition().y << std::endl; break;
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

  std::ofstream geojson_file("example.geojson", std::ios::out |
std::ios::trunc); if (geojson_file.is_open()) { geojson_file << geojson_str;
    geojson_file.close();
  } else {
    std::cerr << "Not open" << std::endl;
  }
  //XodrGeojsonConverter::Convert(buffer.str());
  return 0;
}
*/