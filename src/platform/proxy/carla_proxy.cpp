/*
 * File: proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:11:52 pm
 */

#include "platform/proxy/carla_proxy.h"

using namespace mellocolate;
using namespace mellocolate::utils;
using namespace xviz;

// For readable seconds
using namespace std::chrono_literals;
using namespace std::string_literals;

// For websocket
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

void FlatVector(std::vector<double>& v, const std::vector<double>& to_add, int neg_factor = 1) {
  v.push_back(to_add[0]);
  v.push_back(neg_factor * to_add[1]);
  v.push_back(to_add[2]);
}

std::pair<double, double> AfterRotate(double x, double y, double yaw) {
  return {std::cos(yaw) * x - std::sin(yaw) * y,
          std::sin(yaw) * x + std::cos(yaw) * y};
}

template<typename T>
void AddVerticesToVector(std::vector<std::vector<double>>& v, const boost::shared_ptr<T>& ptr) {
  auto bounding_box = ptr->GetBoundingBox();
  double x_off = bounding_box.extent.x;
  double y_off = bounding_box.extent.y;
  carla::geom::Vector3D bounding_box_pos_0(-x_off, -y_off, 0);
  carla::geom::Vector3D bounding_box_pos_1(-x_off, y_off, 0);
  carla::geom::Vector3D bounding_box_pos_2(x_off, y_off, 0);
  carla::geom::Vector3D bounding_box_pos_3(x_off, -y_off, 0);
  auto transform = ptr->GetTransform();
  transform.TransformPoint(bounding_box_pos_0);
  transform.TransformPoint(bounding_box_pos_1);
  transform.TransformPoint(bounding_box_pos_2);
  transform.TransformPoint(bounding_box_pos_3);
  // double yaw = ptr->GetTransform().rotation.yaw / 180.0 * M_PI;
  // std::vector<std::pair<double, double>> offset = {
  //     AfterRotate(-x_off, -y_off, yaw), AfterRotate(-x_off, y_off, yaw),
  //     AfterRotate(x_off, y_off, yaw), AfterRotate(x_off, -y_off, yaw)};
  // double x = ptr->GetLocation().x;
  // double y = ptr->GetLocation().y;
  // double z = ptr->GetLocation().z;
  std::vector<carla::geom::Vector3D> offset = {
    bounding_box_pos_0,
    bounding_box_pos_1,
    bounding_box_pos_2,
    bounding_box_pos_3
  };
  std::vector<double> vv;
  for (int j = 0; j < offset.size(); j++) {
    vv.push_back(offset[j].x);
    vv.push_back(-offset[j].y);
    vv.push_back(offset[j].z);
    // vv.push_back(x + offset[j].first);
    // vv.push_back(-(y + offset[j].second));
    // vv.push_back(z);
  }
  v.push_back(std::move(vv));
}

void AddMap(nlohmann::json& json, std::string& map) {
  json["map"] = std::move(map);
}

std::unordered_map<std::string, XVIZUIBuilder> GetUIs() {
  std::unordered_map<std::string, XVIZUIBuilder> ui_builders;
  std::vector<std::string> cameras = {"/camera/images"};
  std::vector<std::string> acceleration_stream = {"/vehicle/acceleration"};
  std::vector<std::string> velocity_stream = {"/vehicle/velocity"};
  XVIZVideoBuilder camera_builder(cameras);
  std::shared_ptr<XVIZBaseUIBuilder> metric_builder2 = std::make_shared<XVIZMetricBuilder>(velocity_stream, "velocity", "velocity");

  std::shared_ptr<XVIZContainerBuilder> container_builder = std::make_shared<XVIZContainerBuilder>("metrics", LayoutType::VERTICAL);
  container_builder->Child(acceleration_stream, "acceleration", "acceleration");
  container_builder->Child(metric_builder2);
  container_builder->Child(acceleration_stream, "test", "test");
  ui_builders["Camera"] = XVIZUIBuilder();
  ui_builders["Camera"].Child(camera_builder);
  ui_builders["Metrics"] = XVIZUIBuilder();
  ui_builders["Metrics"].Child(std::move(*container_builder));
  return ui_builders;
}

CarlaProxy::CarlaProxy(boost::shared_ptr<carla::client::Client> client_ptr)
    : client_ptr_(std::move(client_ptr)) {
  world_ptr_ =
      boost::make_shared<carla::client::World>(client_ptr_->GetWorld());
}
CarlaProxy::CarlaProxy(const std::string& carla_host, uint16_t carla_port)
    : carla_host_(carla_host), carla_port_(carla_port) {}


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
}

void CarlaProxy::Clear() {
  client_ptr_->SetTimeout(500ms);
  for (const auto&[id, dummy_sensor] : dummy_sensors_) {
    if (dummy_sensor->IsAlive()) {
      LOG_INFO("Stop listening sensor: %u", id);
      dummy_sensor->Destroy();
    }
  }
  LOG_INFO("Carla proxy clear!");
}

void CarlaProxy::UpdateMetaData() {
  std::string map_geojson =
      utils::XodrGeojsonConverter::GetGeoJsonFromCarlaMap(world_ptr_->GetMap());
    XVIZMetadataBuilder xviz_metadata_builder;
    xviz_metadata_builder.Stream("/vehicle_pose").Category(Category::StreamMetadata_Category_POSE)
        .Stream("/object/vehicles")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYGON)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .StreamStyle(
            "{"
              "\"extruded\": true,"
              "\"fill_color\": \"#40E0D0\","
              "\"height\": 2.0"
            "}")
        .Stream("/object/walkers")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYGON)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .StreamStyle(
            "{"
              "\"extruded\": true,"
              "\"fill_color\": \"#FF69B4\","
              "\"height\": 1.5"
            "}")
        .Stream("/vehicle/acceleration")
          .Category(Category::StreamMetadata_Category_TIME_SERIES)
          .Unit("m/s^2")
          .Type(ScalarType::StreamMetadata_ScalarType_FLOAT)
        .Stream("/vehicle/velocity")
          .Category(Category::StreamMetadata_Category_TIME_SERIES)
          .Unit("m/s")
          .Type(ScalarType::StreamMetadata_ScalarType_FLOAT)
        .Stream("/traffic_lights/red")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYGON)
          .StreamStyle(
            "{"
              "\"extruded\": true,"
              "\"fill_color\": \"#FF0000\","
              "\"height\": 0.1"
            "}")
        .Stream("/traffic_lights/yellow")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYGON)
          .StreamStyle(
            "{"
              "\"extruded\": true,"
              "\"fill_color\": \"#FFFF00\","
              "\"height\": 0.1"
            "}")
        .Stream("/traffic_lights/green")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYGON)
          .StreamStyle(
            "{"
              "\"extruded\": true,"
              "\"fill_color\": \"#00FF00\","
              "\"height\": 0.1"
            "}")
        .Stream("/camera/images")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_IMAGE)
        .Stream("/lidar/points")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_POINT)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .StreamStyle(
            "{"
              "\"point_cloud_mode\": \"distance_to_vehicle\","
              "\"radius_pixels\": 2.0"
            "}")
        .Stream("/drawing/polylines")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYLINE)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .StreamStyle(
            "{"
              "\"stroke_color\": \"#FFD700\","
              "\"stroke_width\": 2.0"
            "}")
        .Stream("/drawing/points")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_POLYLINE)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
        .Stream("/drawing/texts")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_TEXT)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
        .UI(GetUIs());
        
          
  // XVIZMetaDataBuilder xviz_metadata_builder;
  //     .AddStream(
  //         metadata::Stream("/planning/trajectory")
  //             .AddCategory("primitive")
  //             .AddCoordinate("IDENTITY")
  //             .AddStreamStyle(
  //                 metadata::StreamStyle().AddStrokeWidth(2.0).AddStrokeColor(
  //                     "#FFD700"))
  //             .AddType("polyline"))
  metadata_ptr_ = xviz_metadata_builder.GetData();
  auto json = xviz_metadata_builder.GetMessage().ToObject();
  // auto v = ;
  AddMap(json, map_geojson);
  metadata_str_ = json.dump();
  // return xviz_metadata_builder.GetMessage().ToObjectString();
}

std::string CarlaProxy::GetMetaData() {
  return metadata_str_;
}

XVIZBuilder CarlaProxy::GetUpdateData() {
  // auto world_snapshots = world_ptr_->WaitForTick(2s);
  // return GetUpdateData(world_snapshots);
  std::lock_guard lock_guard(internal_update_builder_lock_);
  // xviz::XVIZBuilder builder{metadata_ptr_};
  return internal_update_builder_;
}

void CarlaProxy::UpdateData() {
  while (true) {
    auto world_snapshots = world_ptr_->WaitForTick(2s);
    std::lock_guard lock_guard(internal_update_builder_lock_);
    internal_update_builder_ = GetUpdateData(world_snapshots);
  }
}

XVIZBuilder CarlaProxy::GetUpdateData(
    const carla::client::WorldSnapshot& world_snapshots) {
  std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::system_clock::now();
  double now_time = now.time_since_epoch().count() / 1e9;

  XVIZBuilder xviz_builder(metadata_ptr_);

  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Actor>>
      tmp_actors;
  std::unordered_set<uint32_t> tmp_real_sensors;

  ego_actor_ = nullptr;
  ego_id_ = -1;

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
      if (attribute.GetId() == "role_name" && (attribute.GetValue() == "ego") || attribute.GetValue() == "hero") {
        ego_actor_ = actor_ptr;
        ego_id_ = ego_actor_->GetId();
        need_continue = true;
        break;
      }
    }
    tmp_actors.insert({id, actor_ptr});
    if (need_continue) {
      continue;
    }
    if (actor_ptr->GetTypeId().substr(0, 6) == "sensor") {
      auto sensor_ptr =
          boost::static_pointer_cast<carla::client::Sensor>(actor_ptr);
      // if (real_sensors_.find(id) == real_sensors_.end() &&
      //     dummy_sensors_.find(id) == dummy_sensors_.end()) {
      if (real_sensors_.find(id) == real_sensors_.end() &&
             recorded_dummy_sensor_ids_.find(id) == recorded_dummy_sensor_ids_.end()) {
        if (!sensor_ptr->IsAlive()) {
          continue;
        }
        LOG_INFO("Listen sensor: %u, type is: %s", id,
                 actor_ptr->GetTypeId().c_str());
        auto dummy_sensor = CreateDummySensor(sensor_ptr);
        if (dummy_sensor == nullptr) {
          continue;
        }
        auto dummy_id = dummy_sensor->GetId();
        recorded_dummy_sensor_ids_.insert(dummy_id);
        dummy_sensors_.insert({id, dummy_sensor});
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
                image_data_queues_[id] = std::move(encoded_image);
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
                lidar_data_queues_[id].push_back(std::move(point_cloud));
                lidar_data_lock_.unlock();
                return;
              }
            });
        real_dummy_sensors_relation_.insert({id, dummy_id});
      }
      if (recorded_dummy_sensor_ids_.find(id) == recorded_dummy_sensor_ids_.end()) {
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
    // auto dummy_id = real_dummy_sensors_relation_[id];
    // recorded_dummy_sensor_ids_.erase(dummy_id);
    dummy_sensors_[id]->Stop();
    dummy_sensors_[id]->Destroy();
    dummy_sensors_.erase(id);
    real_dummy_sensors_relation_.erase(id);
    real_sensors_.erase(id);

    image_data_lock_.lock();
    if (image_data_queues_.find(id) != image_data_queues_.end()) {
      image_data_queues_.erase(id);
      is_image_received_ = true;
    }
    image_data_lock_.unlock();

    lidar_data_lock_.lock();
    lidar_data_queues_.erase(id);
    lidar_data_lock_.unlock();
  }
  real_sensors_ = std::move(tmp_real_sensors);

  point_3d_t ego_position(0, 0, 0);
  point_3d_t ego_orientation(0, 0, 0);
  double display_velocity = 0;
  double display_acceleration = 0;

  if (ego_actor_ != nullptr) {
    auto location = ego_actor_->GetLocation();
    auto orientation = ego_actor_->GetTransform().rotation;
    ego_position.set<0>(location.x);
    ego_position.set<1>(-location.y);
    ego_position.set<2>(location.z);
    ego_orientation.set<0>(orientation.roll / 180.0 * M_PI);
    ego_orientation.set<1>(-orientation.pitch / 180.0 * M_PI);
    ego_orientation.set<2>(-(orientation.yaw) / 180.0 * M_PI);

    display_velocity = Utils::ComputeSpeed(ego_actor_->GetVelocity());
    display_acceleration = Utils::ComputeSpeed(ego_actor_->GetAcceleration());
  }

  xviz_builder.Pose("/vehicle_pose")
    .MapOrigin(0, 0, 0)
    .Orientation(ego_orientation.get<0>(), ego_orientation.get<1>(), ego_orientation.get<2>())
    .Position(ego_position.get<0>(), ego_position.get<1>(), ego_position.get<2>())
    .Timestamp(now_time);

  xviz_builder.TimeSeries("/vehicle/acceleration")
    .Timestamp(now_time)
    .Value(display_acceleration)
    .Id("acceleration");

  xviz_builder.TimeSeries("/vehicle/velocity")
    .Timestamp(now_time)
    .Value(display_velocity)
    .Id("velocity");


  std::vector<std::vector<double>> vehicle_vector;
  std::vector<uint32_t> vehicle_ids;
  std::vector<std::vector<double>> walker_vector;
  std::vector<uint32_t> walker_ids;

  for (const auto& actor_pair : actors_) {
    if (ego_id_ == actor_pair.first) {
      continue;
    }
    auto actor_ptr = actor_pair.second;

    if (Utils::IsStartWith(actor_ptr->GetTypeId(), "vehicle")) {
      AddVerticesToVector<carla::client::Vehicle>(vehicle_vector, boost::static_pointer_cast<carla::client::Vehicle>(actor_ptr));
      vehicle_ids.push_back(actor_pair.first);
      continue;
    }
    if (Utils::IsStartWith(actor_ptr->GetTypeId(), "walker")) {
      AddVerticesToVector<carla::client::Walker>(walker_vector, boost::static_pointer_cast<carla::client::Walker>(actor_ptr));
      walker_ids.push_back(actor_pair.first);
      continue;
    }
    if (Utils::IsStartWith(actor_ptr->GetTypeId(), "traffic.traffic_light")) {
      auto traffic_light =
          boost::static_pointer_cast<carla::client::TrafficLight>(actor_ptr);
      auto state = traffic_light->GetState();
      switch (state) {
        case carla::rpc::TrafficLightState::Red:
          AddTrafficLights(xviz_builder.Primitive("/traffic_lights/red"),
                           traffic_light);
          break;
        case carla::rpc::TrafficLightState::Yellow:
          AddTrafficLights(xviz_builder.Primitive("/traffic_lights/yellow"),
                           traffic_light);
          break;
        case carla::rpc::TrafficLightState::Green:
          AddTrafficLights(xviz_builder.Primitive("/traffic_lights/green"),
                           traffic_light);
          break;
        default:
          LOG_WARNING("unknown traffic light state");
      }
      continue;
    }
  }

  for (auto i = 0; i < vehicle_vector.size(); i++) {
    xviz_builder.Primitive("/object/vehicles")
      .Polygon(std::move(vehicle_vector[i]))
      .ObjectId(std::to_string(vehicle_ids[i]))
      .Classes({"vehicle"});
  }

  for (auto i = 0; i < walker_vector.size(); i++) {
    xviz_builder.Primitive("/object/walkers")
      .Polygon(std::move(walker_vector[i]))
      .ObjectId(std::to_string(walker_ids[i]))
      .Classes({"walker"});
  }

  image_data_lock_.lock();
  if (is_image_received_) {
    last_received_images_.clear();
    is_image_received_ = false;
    std::vector<uint32_t> to_delete_image_ids;
    for (const auto& [camera_id, image] : image_data_queues_) {
      if (real_dummy_sensors_relation_.find(camera_id) !=
          real_dummy_sensors_relation_.end()) {
        last_received_images_.push_back(image.GetData());
      } else {
        to_delete_sensor_ids.push_back(camera_id);
      }
    }
    for (auto image_id : to_delete_image_ids) {
      image_data_queues_.erase(image_id);
    }
  }
  image_data_lock_.unlock();
  for (auto& image_data : last_received_images_) {
    xviz_builder.Primitive("/camera/images").Image(image_data);
  }

  lidar_data_lock_.lock();
  XVIZPrimitiveBuilder& point_cloud_builder = xviz_builder.Primitive("/lidar/points");
  std::vector<double> points;
  for (auto& [lidar_id, point_cloud_queue] : lidar_data_queues_) {
    for (auto& point_cloud : point_cloud_queue) {
      points.insert(points.end(), point_cloud.GetPoints().begin(), point_cloud.GetPoints().end());
    }
  }
  lidar_data_lock_.unlock();


  if (!points.empty()) {
    point_cloud_builder.Points(std::move(points));
  }

  return xviz_builder;  //.GetData();
}

void CarlaProxy::AddTrafficLights(
    XVIZPrimitiveBuilder& xviz_primitive_builder,
    boost::shared_ptr<carla::client::TrafficLight> traffic_light) {
  auto id = traffic_light->GetId();
  if (traffic_lights_.find(id) == traffic_lights_.end()) {
    LOG_WARNING("all traffic lights should be inserted first");
    return;
  }

  for (auto& polygon : traffic_lights_[id]) {
    xviz_primitive_builder.Polygon(polygon);
  }
}

void CarlaProxy::AddTrafficLightAreas() {
  auto actor_snapshots = world_ptr_->WaitForTick(2s);
  auto map = world_ptr_->GetMap();
  const double area_length = 2.0;
  for (const auto& actor_snapshot : actor_snapshots) {
    auto actor = world_ptr_->GetActor(actor_snapshot.id);
    if (actor == nullptr) {
      continue;
    }
    if (Utils::IsStartWith(actor->GetTypeId(), "traffic.traffic_light")) {
      auto id = actor->GetId();
      if (traffic_lights_.find(id) == traffic_lights_.end()) {
        traffic_lights_.insert({id, std::vector<std::vector<double>>()});
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
          std::vector<double> area;
          auto width = now_waypoint->GetLaneWidth();
          auto tmp_waypoint = tmp_waypoints[0];
          auto right_top_p = XodrGeojsonConverter::LateralShiftGetVector(
              tmp_waypoint->GetTransform(), width / 2.0);
          auto left_top_p = XodrGeojsonConverter::LateralShiftGetVector(
              tmp_waypoint->GetTransform(), -width / 2.0);
          auto right_down_p = XodrGeojsonConverter::LateralShiftGetVector(
              now_waypoint->GetTransform(), width / 2.0);
          auto left_down_p = XodrGeojsonConverter::LateralShiftGetVector(
              now_waypoint->GetTransform(), -width / 2.0);
          FlatVector(area, right_top_p, -1);
          FlatVector(area, right_down_p, -1);
          FlatVector(area, left_down_p, -1);
          FlatVector(area, left_top_p, -1);
          traffic_lights_[id].push_back(std::move(area));
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
          std::vector<double> area;
          auto width = now_waypoint->GetLaneWidth();
          auto tmp_waypoint = tmp_waypoints[0];
          auto right_top_p = XodrGeojsonConverter::LateralShiftGetVector(
              tmp_waypoint->GetTransform(), width / 2.0);
          auto left_top_p = XodrGeojsonConverter::LateralShiftGetVector(
              tmp_waypoint->GetTransform(), -width / 2.0);
          auto right_down_p = XodrGeojsonConverter::LateralShiftGetVector(
              now_waypoint->GetTransform(), width / 2.0);
          auto left_down_p = XodrGeojsonConverter::LateralShiftGetVector(
              now_waypoint->GetTransform(), -width / 2.0);
          FlatVector(area, right_top_p, -1);
          FlatVector(area, right_down_p, -1);
          FlatVector(area, left_down_p, -1);
          FlatVector(area, left_top_p, -1);
          traffic_lights_[id].push_back(std::move(area));
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
  std::vector<double> points;
  double yaw = lidar_measurement.GetSensorTransform().rotation.yaw;
  auto location = lidar_measurement.GetSensorTransform().location;
  for (const auto& point : lidar_measurement) {
    point_3d_t offset = Utils::GetOffsetAfterTransform(
        point_3d_t(point.x, point.y, point.z), (yaw + 90.0) / 180.0 * M_PI);
    points.emplace_back(location.x + offset.get<0>());
    points.emplace_back(-(location.y + offset.get<1>()));
    points.emplace_back(location.z - offset.get<2>());
  }
  return utils::PointCloud(lidar_measurement.GetTimestamp(), std::move(points));
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
  std::string data_str;
  for (const auto& c : image_data) {
    data_str += (char)c;
  }
  return utils::Image(std::move(data_str), image.GetWidth(), image.GetHeight());
}