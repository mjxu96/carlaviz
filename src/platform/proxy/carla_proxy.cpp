/*
 * File: proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:11:52 pm
 */

#include "platform/proxy/carla_proxy.h"

using namespace mellocolate;
using namespace mellocolate::utils;
// For readable seconds
using namespace std::chrono_literals;
using namespace std::string_literals;

// For websocket
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

std::pair<double, double> AfterRotate(double x, double y, double yaw) {
  return {std::cos(yaw) * x - std::sin(yaw) * y,
          std::sin(yaw) * x + std::cos(yaw) * y};
}

CarlaProxy::CarlaProxy(boost::shared_ptr<carla::client::Client> client_ptr)
    : client_ptr_(std::move(client_ptr)) {
  world_ptr_ =
      boost::make_shared<carla::client::World>(client_ptr_->GetWorld());
}
CarlaProxy::CarlaProxy(const std::string& carla_host, uint16_t carla_port)
    : carla_host_(carla_host), carla_port_(carla_port) {}

void CarlaProxy::Run() {
  while (true) {
    auto world_snapshots = world_ptr_->WaitForTick(2s);
    auto xviz_builder = GetUpdateData(world_snapshots);
    Update(xviz_builder.GetData());  // GetUpdateData(world_snapshots));
  }
}

void CarlaProxy::AddClient(boost::asio::ip::tcp::socket socket) {
  auto ws_ptr =
      boost::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
  try {
    ws_ptr->accept();
    LOG_INFO("Frontend connected");
    boost::beast::multi_buffer buffer;
    boost::beast::ostream(buffer) << GetMetaData();
    ws_ptr->write(buffer.data());

    std::lock_guard<std::mutex> lock_guard(clients_addition_lock_);
    ws_ptrs_.insert(ws_ptr);
  } catch (boost::system::system_error const& se) {
    if (se.code() != websocket::error::closed) {
      throw se;
    } else {
      LOG_INFO("Frontend connection closed");
    }
  }
}

void CarlaProxy::Init() {
  try {
    LOG_INFO("Connecting to Carla Server on %s:%u...", carla_host_.c_str(),
             carla_port_);
    client_ptr_ =
        boost::make_shared<carla::client::Client>(carla_host_, carla_port_);
    if (client_ptr_ == nullptr) {
      LOG_ERROR("Carla client ptr is null. Exiting");
      return;
    } else {
      client_ptr_->SetTimeout(10s);
      std::string server_version = client_ptr_->GetServerVersion();
      LOG_INFO("Connected to Carla Server, Server version is: %s",
               server_version.c_str());
    }

    world_ptr_ =
        boost::make_shared<carla::client::World>(client_ptr_->GetWorld());

    AddTrafficLightAreas();
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
    exit(1);
  }
  // try {
  //   ws_ptr_->accept();
  //   LOG_INFO("Frontend connected");
  //   boost::beast::multi_buffer buffer;
  //   boost::beast::ostream(buffer) << GetMetaData();
  //   ws_ptr_->write(buffer.data());
  // } catch (boost::system::system_error const& se) {
  //   if (se.code() != websocket::error::closed) {
  //     throw se;
  //   } else {
  //     LOG_INFO("Frontend connection closed");
  //   }
  // }
}

void CarlaProxy::Update(const std::string& data_str) {
  boost::beast::multi_buffer buffer;

  boost::beast::ostream(buffer) << data_str;  // GetUpdateData();
  auto data = buffer.data();

  boost::unordered_set<boost::shared_ptr<
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>>
      to_delete_ws_ptrs;

  std::lock_guard<std::mutex> lock_guard(clients_addition_lock_);
  for (auto ws_ptr : ws_ptrs_) {
    try {
      ws_ptr->write(data);
    } catch (boost::system::system_error const& se) {
      to_delete_ws_ptrs.insert(ws_ptr);
      if (se.code() != websocket::error::closed &&
          std::strcmp(se.what(), "Broken pipe") != 0 &&
          std::strcmp(se.what(), "Connection reset by peer")) {
        LOG_ERROR("ERROR WHEN SENDDING UPDATE %s", se.what());
      } else {
        LOG_INFO("Frontend connection closed");
      }
    }
  }

  for (auto to_delete_ws_ptr : to_delete_ws_ptrs) {
    ws_ptrs_.erase(to_delete_ws_ptr);
  }
}

std::string CarlaProxy::GetMetaData() {
  std::string map_geojson =
      utils::XodrGeojsonConverter::GetGeoJsonFromCarlaMap(world_ptr_->GetMap());
  XVIZMetaDataBuilder xviz_metadata_builder;
  xviz_metadata_builder.SetMap(map_geojson)
      .AddStream(metadata::Stream("/vehicle_pose").AddCategory("pose"))
      .AddStream(metadata::Stream("/object/vehicles")
                     .AddCategory("primitive")
                     .AddCoordinate("IDENTITY")
                     .AddStreamStyle(metadata::StreamStyle()
                                         .AddExtruded(true)
                                         .AddFillColor("#fb0")
                                         .AddHeight(2.0))
                     .AddType("polygon"))
      .AddStream(
          metadata::Stream("/planning/trajectory")
              .AddCategory("primitive")
              .AddCoordinate("IDENTITY")
              .AddStreamStyle(
                  metadata::StreamStyle().AddStrokeWidth(2.0).AddStrokeColor(
                      "#00FF00"))
              .AddType("polyline"))
      .AddStream(metadata::Stream("/object/walkers")
                     .AddCategory("primitive")
                     .AddCoordinate("IDENTITY")
                     .AddStreamStyle(metadata::StreamStyle()
                                         .AddExtruded(true)
                                         .AddFillColor("#FF0000")
                                         .AddHeight(1.5))
                     .AddType("polygon"))
      .AddStream(metadata::Stream("/traffic_lights/red")
                     .AddCategory("primitive")
                     .AddCoordinate("IDENTITY")
                     .AddStreamStyle(metadata::StreamStyle()
                                         .AddExtruded(true)
                                         .AddFillColor("#ff0000")
                                         .AddHeight(0.1)))
      .AddStream(metadata::Stream("/traffic_lights/yellow")
                     .AddCategory("primitive")
                     .AddCoordinate("IDENTITY")
                     .AddStreamStyle(metadata::StreamStyle()
                                         .AddExtruded(true)
                                         .AddFillColor("#ffff00")
                                         .AddHeight(0.1)))
      .AddStream(metadata::Stream("/traffic_lights/green")
                     .AddCategory("primitive")
                     .AddCoordinate("IDENTITY")
                     .AddStreamStyle(metadata::StreamStyle()
                                         .AddExtruded(true)
                                         .AddFillColor("#00ff00")
                                         .AddHeight(0.1)))
      .AddStream(
          metadata::Stream("/lidar/points")
              .AddCategory("primitive")
              .AddCoordinate("IDENTITY")
              .AddType("points")
              .AddStreamStyle(metadata::StreamStyle()
                                  .AddPointCloudMode("distance_to_vehicle")
                                  .AddRadiusPixels(2.0)))
      .AddStream(metadata::Stream("/camera/images")
                     .AddCategory("primitive")
                     .AddType("image"));
  metadata::UIConfig ui_config;
  ui_config.AddCamera("/camera/images");
  xviz_metadata_builder.AddUIConfig(ui_config);
  return xviz_metadata_builder.GetMetaData();
}
XVIZBuilder CarlaProxy::GetUpdateData() {
  auto world_snapshots = world_ptr_->WaitForTick(2s);
  return GetUpdateData(world_snapshots);
}

XVIZBuilder CarlaProxy::GetUpdateData(
    const carla::client::WorldSnapshot& world_snapshots) {
  std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::system_clock::now();
  double now_time = now.time_since_epoch().count() / 1e9;

  XVIZBuilder xviz_builder;
  XVIZPrimitiveBuider xviz_primitive_builder("/object/vehicles");
  XVIZPrimitiveBuider xviz_primitive_walker_builder("/object/walkers");
  XVIZPrimitiveBuider xviz_primitive_traffic_light_red_builder(
      "/traffic_lights/red");
  XVIZPrimitiveBuider xviz_primitive_traffic_light_yellow_builder(
      "/traffic_lights/yellow");
  XVIZPrimitiveBuider xviz_primitive_traffic_light_green_builder(
      "/traffic_lights/green");

  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Actor>>
      tmp_actors;
  std::unordered_set<uint32_t> tmp_real_sensors;

  ego_actor_ = nullptr;

  for (const auto& world_snapshot : world_snapshots) {
    uint32_t id = world_snapshot.id;
    auto actor_it = actors_.find(id);
    boost::shared_ptr<carla::client::Actor> actor_ptr = nullptr;
    if (actor_it == actors_.end()) {
      actor_ptr = world_ptr_->GetActor(id);
    } else {
      actor_ptr = actor_it->second;
    }
    if (actor_ptr == nullptr) {
      LOG_WARNING("Actor pointer is null, actor id: %u", id);
      continue;
    }
    bool need_continue = false;
    for (const auto& attribute : actor_ptr->GetAttributes()) {
      if (attribute.GetId() == "role_name" && attribute.GetValue() == "ego") {
        ego_actor_ = actor_ptr;
        need_continue = true;
      }
    }
    if (need_continue) {
      continue;
    }
    tmp_actors.insert({id, actor_ptr});
    if (actor_ptr->GetTypeId().substr(0, 6) == "sensor") {
      auto sensor_ptr =
          boost::static_pointer_cast<carla::client::Sensor>(actor_ptr);
      if (real_sensors_.find(id) == real_sensors_.end() &&
          dummy_sensors_.find(id) == dummy_sensors_.end()) {
        LOG_INFO("Listen sensor: %u, type is: %s", id,
                 actor_ptr->GetTypeId().c_str());
        auto dummy_sensor = CreateDummySensor(sensor_ptr);
        if (dummy_sensor == nullptr) {
          continue;
        }
        auto dummy_id = dummy_sensor->GetId();
        dummy_sensors_.insert({dummy_id, dummy_sensor});
        double rotation_frequency = 10.0;
        if (utils::Utils::IsStartWith(sensor_ptr->GetTypeId(),
                                      "sensor.lidar")) {
          for (const auto& attribute : sensor_ptr->GetAttributes()) {
            if (attribute.GetId() == "rotation_frequency") {
              rotation_frequency = std::stod(attribute.GetValue());
            }
          }
        }
        dummy_sensor->Listen(
            [this, id, rotation_frequency](
                carla::SharedPtr<carla::sensor::SensorData> data) {
              if (data == nullptr) {
                return;
              }
              auto image_data =
                  boost::dynamic_pointer_cast<carla::sensor::data::Image>(data);
              if (image_data != nullptr) {
                auto encoded_image = this->GetEncodedImage(*image_data);
                image_data_lock_.lock();
                is_image_received_ = true;
                image_data_queues_[id] = encoded_image;
                image_data_lock_.unlock();
                return;
              }
              auto lidar_data = boost::dynamic_pointer_cast<
                  carla::sensor::data::LidarMeasurement>(data);
              if (lidar_data != nullptr) {
                auto point_cloud = this->GetPointCloud(*(lidar_data));
                lidar_data_lock_.lock();
                if (lidar_data_queues_.find(id) == lidar_data_queues_.end()) {
                  lidar_data_queues_[id] = std::deque<utils::PointCloud>();
                }
                if (!lidar_data_queues_[id].empty() &&
                    point_cloud.GetTimestamp() -
                            lidar_data_queues_[id].front().GetTimestamp() >
                        1.0 / rotation_frequency) {
                  lidar_data_queues_[id].pop_front();
                }
                lidar_data_queues_[id].push_back(point_cloud);
                lidar_data_lock_.unlock();
                return;
              }
            });
        real_dummy_sensors_relation_.insert({id, dummy_id});
      }
      if (dummy_sensors_.find(id) == dummy_sensors_.end()) {
        tmp_real_sensors.insert(id);
      }
    }
  }

  actors_ = std::move(tmp_actors);

  std::vector<uint32_t> to_delete_sensor_ids;
  for (const auto& real_sensor_id : real_sensors_) {
    if (tmp_real_sensors.find(real_sensor_id) == tmp_real_sensors.end()) {
      to_delete_sensor_ids.push_back(real_sensor_id);
    }
  }
  for (const auto& id : to_delete_sensor_ids) {
    LOG_INFO("Stop listening sensor: %u", id);
    auto dummy_id = real_dummy_sensors_relation_[id];
    dummy_sensors_[dummy_id]->Stop();
    dummy_sensors_[dummy_id]->Destroy();
    real_dummy_sensors_relation_.erase(id);
    real_sensors_.erase(id);

    image_data_lock_.lock();
    image_data_queues_.erase(id);
    image_data_lock_.unlock();

    lidar_data_lock_.lock();
    lidar_data_queues_.erase(id);
    lidar_data_lock_.unlock();
  }
  real_sensors_ = std::move(tmp_real_sensors);

  point_3d_t ego_position(0, 0, 0);
  point_3d_t ego_orientation(0, 0, 0);

  if (ego_actor_ != nullptr) {
    auto location = ego_actor_->GetLocation();
    auto orientation = ego_actor_->GetTransform().rotation;
    ego_position.set<0>(location.x);
    ego_position.set<1>(-location.y);
    ego_position.set<2>(location.z);
    ego_orientation.set<0>(orientation.roll / 180.0 * M_PI);
    ego_orientation.set<1>(orientation.pitch / 180.0 * M_PI);
    ego_orientation.set<2>(-(orientation.yaw) / 180.0 * M_PI);
  }

  xviz_builder.AddTimestamp(now_time).AddPose(
      XVIZPoseBuilder("/vehicle_pose")
          .AddMapOrigin(point_3d_t(0, 0, 0))
          .AddOrientation(ego_orientation)
          .AddPosition(ego_position)
          .AddTimestamp(now_time));

  for (const auto& actor_pair : actors_) {
    auto actor_ptr = actor_pair.second;

    if (Utils::IsStartWith(actor_ptr->GetTypeId(), "vehicle")) {
      AddVehicle(xviz_primitive_builder,
                 boost::static_pointer_cast<carla::client::Vehicle>(actor_ptr));
    }
    if (Utils::IsStartWith(actor_ptr->GetTypeId(), "walker")) {
      AddWalker(xviz_primitive_walker_builder,
                boost::static_pointer_cast<carla::client::Walker>(actor_ptr));
    }
    if (Utils::IsStartWith(actor_ptr->GetTypeId(), "traffic.traffic_light")) {
      auto traffic_light =
          boost::static_pointer_cast<carla::client::TrafficLight>(actor_ptr);
      auto state = traffic_light->GetState();
      switch (state) {
        case carla::rpc::TrafficLightState::Red:
          AddTrafficLights(xviz_primitive_traffic_light_red_builder,
                           traffic_light);
          break;
        case carla::rpc::TrafficLightState::Yellow:
          AddTrafficLights(xviz_primitive_traffic_light_yellow_builder,
                           traffic_light);
          break;
        case carla::rpc::TrafficLightState::Green:
          AddTrafficLights(xviz_primitive_traffic_light_green_builder,
                           traffic_light);
          break;
        default:
          LOG_WARNING("unknown traffic light state");
      }
    }
  }
  xviz_builder.AddPrimitive(xviz_primitive_builder)
      .AddPrimitive(xviz_primitive_walker_builder)
      .AddPrimitive(xviz_primitive_traffic_light_red_builder)
      .AddPrimitive(xviz_primitive_traffic_light_yellow_builder)
      .AddPrimitive(xviz_primitive_traffic_light_green_builder);

  bool should_add = false;
  XVIZPrimitiveBuider image_builder("/camera/images");
  image_data_lock_.lock();
  if (is_image_received_) {
    std::vector<uint32_t> to_delete_image_ids;
    for (const auto& image_pair : image_data_queues_) {
      if (real_dummy_sensors_relation_.find(image_pair.first) !=
          real_dummy_sensors_relation_.end()) {
        should_add = true;
        image_builder.AddImages(XVIZPrimitiveImageBuilder(image_pair.second));
      } else {
        to_delete_sensor_ids.push_back(image_pair.first);
      }
    }
    for (auto image_id : to_delete_image_ids) {
      image_data_queues_.erase(image_id);
    }
  }
  image_data_lock_.unlock();
  if (should_add) {
    xviz_builder.AddPrimitive(image_builder);
  }

  lidar_data_lock_.lock();
  XVIZPrimitiveBuider point_cloud_builder("/lidar/points");
  for (const auto& point_cloud_pair : lidar_data_queues_) {
    for (const auto& point_cloud : point_cloud_pair.second) {
      point_cloud_builder.AddPoints(
          XVIZPrimitivePointBuilder(point_cloud.GetPoints()));
    }
  }
  lidar_data_lock_.unlock();

  xviz_builder.AddPrimitive(point_cloud_builder);
  return xviz_builder;  //.GetData();
}

void CarlaProxy::AddVehicle(
    XVIZPrimitiveBuider& xviz_primitive_builder,
    boost::shared_ptr<carla::client::Vehicle> vehicle_ptr) {
  auto bounding_box = vehicle_ptr->GetBoundingBox();
  double x_off = bounding_box.extent.x;
  double y_off = bounding_box.extent.y;
  double yaw = vehicle_ptr->GetTransform().rotation.yaw / 180.0 * M_PI;
  std::vector<std::pair<double, double>> offset = {
      AfterRotate(-x_off, -y_off, yaw), AfterRotate(-x_off, y_off, yaw),
      AfterRotate(x_off, y_off, yaw), AfterRotate(x_off, -y_off, yaw)};
  double x = vehicle_ptr->GetLocation().x;
  double y = vehicle_ptr->GetLocation().y;
  double z = vehicle_ptr->GetLocation().z;
  std::vector<point_3d_t> vertices;
  for (int j = 0; j < offset.size(); j++) {
    vertices.emplace_back(x + offset[j].first, -(y + offset[j].second), z);
  }
  xviz_primitive_builder.AddPolygon(XVIZPrimitivePolygonBuilder(vertices).AddId(
      vehicle_ptr->GetTypeId() + std::string(".") +
      std::to_string(vehicle_ptr->GetId())));
}

void CarlaProxy::AddWalker(
    XVIZPrimitiveBuider& xviz_primitive_builder,
    boost::shared_ptr<carla::client::Walker> walker_ptr) {
  auto bounding_box = walker_ptr->GetBoundingBox();
  double x_off = bounding_box.extent.x;
  double y_off = bounding_box.extent.y;
  double yaw = walker_ptr->GetTransform().rotation.yaw / 180.0 * M_PI;
  std::vector<std::pair<double, double>> offset = {
      AfterRotate(-x_off, -y_off, yaw), AfterRotate(-x_off, y_off, yaw),
      AfterRotate(x_off, y_off, yaw), AfterRotate(x_off, -y_off, yaw)};
  double x = walker_ptr->GetLocation().x;
  double y = walker_ptr->GetLocation().y;
  double z = walker_ptr->GetLocation().z;
  std::vector<point_3d_t> vertices;
  for (int j = 0; j < offset.size(); j++) {
    vertices.emplace_back(x + offset[j].first, -(y + offset[j].second), z);
  }
  xviz_primitive_builder.AddPolygon(XVIZPrimitivePolygonBuilder(vertices).AddId(
      walker_ptr->GetTypeId() + std::string(".") +
      std::to_string(walker_ptr->GetId())));
}

void CarlaProxy::AddTrafficLights(
    XVIZPrimitiveBuider& xviz_primitive_builder,
    boost::shared_ptr<carla::client::TrafficLight> traffic_light) {
  auto id = traffic_light->GetId();
  if (traffic_lights_.find(id) == traffic_lights_.end()) {
    LOG_WARNING("all traffic lights should be inserted first");
    return;
  }

  for (const auto& polygon : traffic_lights_[id]) {
    xviz_primitive_builder.AddPolygon(XVIZPrimitivePolygonBuilder(polygon));
  }
  // auto trigger = traffic_light->GetTriggerVolume();
  // double x_off = trigger.extent.x;
  // double y_off = trigger.extent.y;
  // auto transform = traffic_light->GetTransform();
  // transform.TransformPoint(trigger.location);
  // auto location = trigger.location;
  // std::vector<point_3d_t> vertices;

  // double yaw = transform.rotation.yaw / 180.0 * M_PI;
  // std::vector<std::pair<double, double>> offset = {
  //     AfterRotate(-x_off, -y_off, yaw), AfterRotate(-x_off, y_off, yaw),
  //     AfterRotate(x_off, y_off, yaw), AfterRotate(x_off, -y_off, yaw)};

  // for (int j = 0; j < offset.size(); j++) {
  //   vertices.emplace_back(location.x + offset[j].first, -(location.y +
  //   offset[j].second), location.z);
  // }
  // xviz_primitive_builder.AddPolygon(XVIZPrimitivePolygonBuilder(vertices));
}

void dbgPrintMaxMinDeg(const std::vector<point_3d_t>& points) {
  double min_deg = 10000;
  double max_deg = -11111;
  for (const auto& point : points) {
    double x = point.get<0>();
    double y = point.get<1>();
    double deg = std::atan2(y, x) / M_PI * 180.0;
    if (x < 0) {
      deg += 180.0;
    }
    min_deg = std::min(min_deg, deg);
    max_deg = std::max(max_deg, deg);
  }
  LOG_INFO("MIN: %.2f, MAX: %.2f", min_deg, max_deg);
}

std::string dbgPointString(const point_3d_t& p) {
  std::string str = "[" + std::to_string(p.get<0>()) + ", " +
                    std::to_string(p.get<1>()) + "]";
  return str;
}

std::string dbgPolygonString(const std::vector<point_3d_t>& pol) {
  std::string str;
  for (const auto& p : pol) {
    str += dbgPointString(p) + ", ";
  }
  return str;
}

void CarlaProxy::AddTrafficLightAreas() {
  auto actors = world_ptr_->GetActors();
  auto map = world_ptr_->GetMap();
  const double area_length = 2.0;
  for (const auto& actor : *actors) {
    if (Utils::IsStartWith(actor->GetTypeId(), "traffic.traffic_light")) {
      auto id = actor->GetId();
      if (traffic_lights_.find(id) == traffic_lights_.end()) {
        traffic_lights_.insert({id, std::vector<std::vector<point_3d_t>>()});
      }
      auto tl = boost::static_pointer_cast<carla::client::TrafficLight>(actor);
      auto trigger_volume = tl->GetTriggerVolume();
      auto transform = tl->GetTransform();
      transform.TransformPoint(trigger_volume.location);
      double x_off = trigger_volume.extent.x;
      double y_off = trigger_volume.extent.y;
      std::vector<point_3d_t> vertices;
      double yaw = transform.rotation.yaw / 180.0 * M_PI;
      auto location = trigger_volume.location;
      std::vector<std::pair<double, double>> offset = {
          AfterRotate(-x_off, -y_off, yaw), AfterRotate(-x_off, y_off, yaw),
          AfterRotate(x_off, y_off, yaw), AfterRotate(x_off, -y_off, yaw)};

      for (int j = 0; j < offset.size(); j++) {
        vertices.emplace_back(location.x + offset[j].first,
                              (location.y + offset[j].second), location.z);
      }

      auto central_waypoint = map->GetWaypoint(location);
      auto now_waypoint = central_waypoint;
      std::unordered_set<uint32_t> visited_points;
      while (now_waypoint != nullptr) {
        auto now_id = now_waypoint->GetId();
        auto lane_type = now_waypoint->GetType();
        if (visited_points.find(now_id) != visited_points.end()) {
          break;
        }
        visited_points.insert(now_id);
        if (lane_type != carla::road::Lane::LaneType::Driving) {
          now_waypoint = now_waypoint->GetLeft();
          continue;
        }
        auto loc = now_waypoint->GetTransform().location;
        point_3d_t p(loc.x, loc.y, loc.z);
        if (!Utils::IsWithin(p, vertices)) {
          break;
        }
        auto tmp_waypoints = now_waypoint->GetNext(area_length);
        if (tmp_waypoints.empty()) {
          LOG_WARNING(
              "the waypoint of the trigger volume of a traffic light is too "
              "close to the intersection, the map does not show the volumn");
        } else {
          std::vector<point_3d_t> area;
          auto width = now_waypoint->GetLaneWidth();
          auto tmp_waypoint = tmp_waypoints[0];
          auto right_top_p = XodrGeojsonConverter::LateralShift(
              tmp_waypoint->GetTransform(), width / 2.0);
          auto left_top_p = XodrGeojsonConverter::LateralShift(
              tmp_waypoint->GetTransform(), -width / 2.0);
          auto right_down_p = XodrGeojsonConverter::LateralShift(
              now_waypoint->GetTransform(), width / 2.0);
          auto left_down_p = XodrGeojsonConverter::LateralShift(
              now_waypoint->GetTransform(), -width / 2.0);
          area.push_back(right_top_p);
          area.push_back(right_down_p);
          area.push_back(left_down_p);
          area.push_back(left_top_p);
          for (auto& p : area) {
            p.set<1>(-p.get<1>());
          }
          traffic_lights_[id].push_back(area);
        }
        now_waypoint = now_waypoint->GetLeft();
      }

      // now go through right points
      now_waypoint = central_waypoint->GetRight();
      visited_points.clear();
      while (now_waypoint != nullptr) {
        auto now_id = now_waypoint->GetId();
        auto lane_type = now_waypoint->GetType();
        if (visited_points.find(now_id) != visited_points.end()) {
          std::cout << "alread searched" << std::endl;
          break;
        }
        visited_points.insert(now_id);
        if (lane_type != carla::road::Lane::LaneType::Driving) {
          now_waypoint = now_waypoint->GetRight();
          continue;
        }
        auto loc = now_waypoint->GetTransform().location;
        point_3d_t p(loc.x, loc.y, loc.z);
        if (!Utils::IsWithin(p, vertices)) {
          break;
        }
        auto tmp_waypoints = now_waypoint->GetNext(area_length);
        if (tmp_waypoints.empty()) {
          LOG_WARNING(
              "the waypoint of the trigger volume of a traffic light is too "
              "close to the intersection, the map does not show the volumn");
        } else {
          std::vector<point_3d_t> area;
          auto width = now_waypoint->GetLaneWidth();
          auto tmp_waypoint = tmp_waypoints[0];
          auto right_top_p = XodrGeojsonConverter::LateralShift(
              tmp_waypoint->GetTransform(), width / 2.0);
          auto left_top_p = XodrGeojsonConverter::LateralShift(
              tmp_waypoint->GetTransform(), -width / 2.0);
          auto right_down_p = XodrGeojsonConverter::LateralShift(
              now_waypoint->GetTransform(), width / 2.0);
          auto left_down_p = XodrGeojsonConverter::LateralShift(
              now_waypoint->GetTransform(), -width / 2.0);
          area.push_back(right_top_p);
          area.push_back(right_down_p);
          area.push_back(left_down_p);
          area.push_back(left_top_p);
          for (auto& p : area) {
            p.set<1>(-p.get<1>());
          }
          traffic_lights_[id].push_back(area);
        }
        now_waypoint = now_waypoint->GetRight();
      }
    }
  }
}

carla::geom::Transform CarlaProxy::GetRelativeTransform(
    const carla::geom::Transform& child, const carla::geom::Transform& parent) {
  auto child_location = child.location;
  auto parent_location = parent.location;
  auto relative_location =
      carla::geom::Location(child_location.x - parent_location.x,
                            child_location.y - parent_location.y,
                            child_location.z - parent_location.z);

  auto child_rotation = child.rotation;
  auto parent_rotation = parent.rotation;
  auto relative_rotation =
      carla::geom::Rotation(child_rotation.pitch - parent_rotation.pitch,
                            child_rotation.yaw - parent_rotation.yaw,
                            child_rotation.roll - parent_rotation.roll);
  return carla::geom::Transform(relative_location, relative_rotation);
}

boost::shared_ptr<carla::client::Sensor> CarlaProxy::CreateDummySensor(
    boost::shared_ptr<carla::client::Sensor> real_sensor) {
  auto real_sensor_attribute = real_sensor->GetAttributes();
  auto type_id = real_sensor->GetTypeId();
  auto blueprint_lib = world_ptr_->GetBlueprintLibrary();
  auto blueprint = (*(blueprint_lib->Filter(type_id)))[0];

  for (const auto& attribute : real_sensor_attribute) {
    blueprint.SetAttribute(attribute.GetId(), attribute.GetValue());
  }

  auto parent = real_sensor->GetParent();
  auto parent_transform = carla::geom::Transform();
  if (parent == nullptr) {
    LOG_WARNING("Real sensor with id %ud has no attached actor",
                real_sensor->GetId());
  } else {
    parent_transform = parent->GetTransform();
  }
  auto sensor_transform = real_sensor->GetTransform();
  auto relative_transform =
      GetRelativeTransform(sensor_transform, parent_transform);

  auto dummy_sensor = boost::static_pointer_cast<carla::client::Sensor>(
      world_ptr_->SpawnActor(blueprint, relative_transform, parent.get()));
  return dummy_sensor;
}

utils::PointCloud CarlaProxy::GetPointCloud(
    const carla::sensor::data::LidarMeasurement& lidar_measurement) {
  std::vector<point_3d_t> points;
  // std::vector<point_3d_t> dbg_points;
  double yaw = lidar_measurement.GetSensorTransform().rotation.yaw;
  auto location = lidar_measurement.GetSensorTransform().location;
  for (const auto& point : lidar_measurement) {
    // dbg_points.emplace_back(point.x, point.y, point.z);
    point_3d_t offset = Utils::GetOffsetAfterTransform(
        point_3d_t(point.x, point.y, point.z), (yaw + 90.0) / 180.0 * M_PI);
    points.emplace_back(location.x + offset.get<0>(),
                        -(location.y + offset.get<1>()),
                        location.z - offset.get<2>());
  }
  // uint32_t partition =
  // (uint32_t)((int)lidar_measurement.GetHorizontalAngle()) / 120;
  return utils::PointCloud(lidar_measurement.GetTimestamp(), points);
}

utils::Image CarlaProxy::GetEncodedImage(
    const carla::sensor::data::Image& image) {
  std::vector<unsigned char> pixel_data;
  for (const auto& p : image) {
    pixel_data.emplace_back(p.r);
    pixel_data.emplace_back(p.g);
    pixel_data.emplace_back(p.b);
    pixel_data.emplace_back(p.a);
  }
  std::vector<unsigned char> image_data;
  unsigned int error = lodepng::encode(image_data, pixel_data, image.GetWidth(),
                                       image.GetHeight());
  if (error) {
    LOG_ERROR("Encoding png error");
  }
  std::string data_str = base64_encode(image_data.data(), image_data.size());
  utils::Image encoded_image(data_str, image.GetWidth(), image.GetHeight());
  return encoded_image;
}