/*
 * File: proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:11:52 pm
 */

#include "backend/proxy/carla_proxy.h"

using namespace carlaviz;
using namespace carlaviz::utils;
using namespace xviz;

// For readable seconds
using namespace std::chrono_literals;
using namespace std::string_literals;

// For websocket
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

std::unordered_map<uint8_t, std::vector<unsigned char>> label_color_map = {
  {0, {0, 0, 0}},
  {1, {70, 70, 70}},
  {2, {190, 153, 153}},
  {3, {250, 170, 160}},
  {4, {220, 20, 60}},
  {5, {153, 153, 153}},
  {6,	{157, 234, 50}},
  {7, {128, 64, 128}},
  {8, {244, 35, 232}},
  {9, {107, 142, 35}},
  {10, {0, 0, 142}},
  {11, {102, 102, 156}},
  {12, {220, 220, 0}}
};
 
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
  }
  v.push_back(std::move(vv));
}

void AddMap(nlohmann::json& json, std::string& map) {
  json["map"] = std::move(map);
}

std::unordered_map<std::string, XVIZUIBuilder> GetUIs(const std::vector<std::string>& cameras,
  const std::vector<std::string>& acceleration_stream, const std::vector<std::string>& velocity_stream,
  const std::vector<std::string>& table_streams) {

  std::unordered_map<std::string, XVIZUIBuilder> ui_builders;

  if (!cameras.empty()) {
    XVIZVideoBuilder camera_builder(cameras);
    ui_builders["Camera"] = XVIZUIBuilder();
    ui_builders["Camera"].Child(camera_builder);
  }

  if (!acceleration_stream.empty() && !velocity_stream.empty()) {
    XVIZContainerBuilder container_builder("metrics", LayoutType::VERTICAL);
    container_builder.Child(acceleration_stream, "acceleration", "acceleration");
    container_builder.Child(velocity_stream, "velocity", "velocity");
    ui_builders["Metrics"] = XVIZUIBuilder();
    ui_builders["Metrics"].Child(container_builder);
  }

  XVIZContainerBuilder tables_container_builder("tables", LayoutType::VERTICAL);
  auto game_time_stream = std::make_shared<XVIZTableBuilder>("Game", "Game time and frame", "/game/time", false);
  tables_container_builder.Child(game_time_stream);
  for (const auto& stream_name : table_streams) {
    auto table_stream = std::make_shared<XVIZTableBuilder>(stream_name, stream_name, stream_name, false);
    tables_container_builder.Child(table_stream);
  }
  ui_builders["Tables"] = XVIZUIBuilder();
  ui_builders["Tables"].Child(tables_container_builder);
  return ui_builders;
}

CarlaProxy::CarlaProxy(boost::shared_ptr<carla::client::Client> client_ptr)
    : client_ptr_(std::move(client_ptr)) {
  world_ptr_ =
      boost::make_shared<carla::client::World>(client_ptr_->GetWorld());
}
CarlaProxy::CarlaProxy(const std::string& carla_host, uint16_t carla_port, bool is_experimental_server_enabled)
    : carla_host_(carla_host), carla_port_(carla_port), 
      is_experimental_server_enabled_(is_experimental_server_enabled) {}


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

void CarlaProxy::SetUpdateMetadataCallback(const std::function<void(const std::string&)>& func) {
  frontend_proxy_update_metadata_callback_ = func;
}

void CarlaProxy::SetTransmissionStreams(const std::unordered_map<std::string, bool>& stream_settings) {
  std::lock_guard lock_g(stream_settings_lock_);
  stream_settings_ = stream_settings;
}

bool CarlaProxy::IsStreamAllowTransmission(const std::string& stream_name) {
  std::lock_guard lock_g(stream_settings_lock_);
  return (stream_settings_.find(stream_name) == stream_settings_.end() || stream_settings_[stream_name]);
}

void CarlaProxy::AddCameraStream(uint32_t camera_id, const std::string& stream_name) {
  std::string name = stream_name;
  if (name.empty()) {
    name = "/camera/" + std::to_string(camera_id);
  }
  LOG_INFO("Add camera stream %u - %s", camera_id, name.c_str());
  camera_streams_[camera_id] = name; 
  UpdateMetadataBuilder();
}

void CarlaProxy::RemoveCameraStream(uint32_t camera_id) {
  if (camera_streams_.find(camera_id) == camera_streams_.end()) {
    LOG_WARNING("Remove invalid camera stream %u", camera_id);
    return;
  }
  LOG_INFO("Remove camera stream %u - %s", camera_id, camera_streams_[camera_id].c_str());
  camera_streams_.erase(camera_id);
  UpdateMetadataBuilder();
}

void CarlaProxy::AddEgoVehicleMetricStreams() {
  UpdateMetadataBuilder();
}

void CarlaProxy::RemoveVehicleMetricStreams() {
  UpdateMetadataBuilder();
}

void CarlaProxy::AddTableStreams(const std::string& sensor_type_name) {
  LOG_INFO("Add sensor stream %s", sensor_type_name.c_str());
  other_sensor_streams_.insert(sensor_type_name);
  UpdateMetadataBuilder();
}

void CarlaProxy::RemoveTableStreams(const std::string& sensor_type_name) {
  if (other_sensor_streams_.find(sensor_type_name) == other_sensor_streams_.end()) {
    LOG_WARNING("Remove invalid sensor stream %s", sensor_type_name.c_str());
    return;
  }
  LOG_INFO("Remove sensor stream %s", sensor_type_name.c_str());
  other_sensor_streams_.erase(sensor_type_name);
  UpdateMetadataBuilder();
}

void CarlaProxy::UpdateMetadataBuilder() {
  auto base_metadata_builder = GetBaseMetadataBuilder();
  std::vector<std::string> camera_streams_vec;
  for (const auto& [c_id, s_name] : camera_streams_) {
    base_metadata_builder
      .Stream(s_name)
        .Category(Category::StreamMetadata_Category_PRIMITIVE)
        .Type(Primitive::StreamMetadata_PrimitiveType_IMAGE);
    camera_streams_vec.push_back(s_name);
  }

  std::vector<std::string> table_streams;
  for (const auto& s_name : other_sensor_streams_) {
    base_metadata_builder
      .Stream(s_name)
        .Category(StreamMetadata::UI_PRIMITIVE);
    table_streams.push_back(s_name);
  }
  if (ego_actor_ != nullptr) {
    base_metadata_builder
      .UI(GetUIs(camera_streams_vec,  {"/vehicle/acceleration"}, {"/vehicle/velocity"}, table_streams));
  } else {
    base_metadata_builder
      .UI(GetUIs(camera_streams_vec,  {}, {}, table_streams));
  }
  metadata_builder_ = base_metadata_builder;
  metadata_ptr_ = metadata_builder_.GetData();
  is_need_update_metadata_ = true;
}

xviz::XVIZMetadataBuilder CarlaProxy::GetBaseMetadataBuilder() {
  XVIZMetadataBuilder xviz_metadata_builder;
    xviz_metadata_builder
        .Stream("/vehicle_pose").Category(Category::StreamMetadata_Category_POSE)
        .Stream("/game/time")
          .Category(StreamMetadata::UI_PRIMITIVE)
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
        .Stream("/lidar/points")
          .Category(Category::StreamMetadata_Category_PRIMITIVE)
          .Type(Primitive::StreamMetadata_PrimitiveType_POINT)
          .Coordinate(CoordinateType::StreamMetadata_CoordinateType_IDENTITY)
          .StreamStyle(
            "{"
              "\"point_color_mode\": \"ELEVATION\","
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
        .UI(GetUIs({}, {}, {}, {}));
  return xviz_metadata_builder;
}

std::string CarlaProxy::GetMapString() {
  return utils::XodrGeojsonConverter::GetGeoJsonFromCarlaMap(world_ptr_->GetMap());
}

std::string CarlaProxy::GetMetadata() {
  // UpdateMetadataBuilder();
  auto xviz_metadata_builder = GetBaseMetadataBuilder();
  metadata_ptr_ = xviz_metadata_builder.GetData();
  return xviz_metadata_builder.GetMessage().ToObjectString();
}

void CarlaProxy::UpdateMetadata() {
  LOG_INFO("Update metadata");
  frontend_proxy_update_metadata_callback_(metadata_builder_.GetMessage().ToObjectString());
}

XVIZBuilder CarlaProxy::GetUpdateData() {
  auto world_snapshots = world_ptr_->WaitForTick(5s);
  return GetUpdateData(world_snapshots);
}

XVIZBuilder CarlaProxy::GetUpdateData(
    const carla::client::WorldSnapshot& world_snapshots) {
  // std::chrono::time_point<std::chrono::system_clock> now =
  //     std::chrono::system_clock::now();
  // double now_time = now.time_since_epoch().count() / 1e9;
  double now_time = world_snapshots.GetTimestamp().elapsed_seconds;
  size_t now_frame = world_snapshots.GetFrame();


  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Actor>>
      tmp_actors;
  std::unordered_set<uint32_t> tmp_real_sensors;

  bool is_ego_found = false;

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
    tmp_actors.insert({id, actor_ptr});
    
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
        auto type_id = actor_ptr->GetTypeId();
        LOG_INFO("Listen sensor: %u, type is: %s", id,
                 type_id.c_str());
        auto dummy_sensor_with_parent_name = CreateDummySensor(sensor_ptr);
        auto parent_name = dummy_sensor_with_parent_name.first;
        auto dummy_sensor = dummy_sensor_with_parent_name.second;
        if (dummy_sensor == nullptr) {
          continue;
        }
        if (utils::Utils::IsStartWith(type_id,
              "sensor.camera")) {
          AddCameraStream(id, "/camera/" + type_id.substr(14) + "/" + std::to_string(id));
        }
        if (utils::Utils::IsStartWith(type_id, "sensor.other.collision")) {
          AddTableStreams("/sensor/other/collision");
          collision_lock_.lock();
          collision_events_[id] = CollisionEvent(0u, 0u, parent_name, "no collision", -1, 0u);
          collision_lock_.unlock();
        }
        if (utils::Utils::IsStartWith(type_id, "sensor.other.gnss")) {
          AddTableStreams("/sensor/other/gnss");
        }
        if (type_id == "sensor.other.obstacle") {
          AddTableStreams("/sensor/other/obstacle");
          obstacle_lock_.lock();
          obstacle_infos_[id] = ObstacleInfo(parent_name, "no obstacle", -1.0, -1.0, 0u);
          obstacle_lock_.unlock();
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
        dummy_sensor->Listen(std::bind(
          &CarlaProxy::HandleSensorData, this, id, rotation_frequency, type_id, parent_name, std::placeholders::_1
        ));
        real_dummy_sensors_relation_.insert({id, dummy_id});
      }
      if (recorded_dummy_sensor_ids_.find(id) == recorded_dummy_sensor_ids_.end()) {
        tmp_real_sensors.insert(id);
      }
      continue;
    }

    if (is_ego_found) {
      continue;
    }
    for (const auto& attribute : actor_ptr->GetAttributes()) {
      if (attribute.GetId() == "role_name" && (attribute.GetValue() == "ego" || attribute.GetValue() == "hero")) {
        ego_actor_ = actor_ptr;
        is_ego_found = true;
        break;
      }
    }
  }

  if (!is_ego_found) {
    ego_actor_ = nullptr;
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
      RemoveCameraStream(id);
      image_data_queues_.erase(id);
      is_image_received_[id] = true;
    }
    image_data_lock_.unlock();

    lidar_data_lock_.lock();
    lidar_data_queues_.erase(id);
    lidar_data_lock_.unlock();

    collision_lock_.lock();
    bool is_previous_empty = collision_events_.empty();
    collision_events_.erase(id);
    if (collision_events_.empty() && !is_previous_empty) {
      RemoveTableStreams("/sensor/other/collision");
    }
    collision_lock_.unlock();

    gnss_lock_.lock();
    is_previous_empty = gnss_infos_.empty();
    gnss_infos_.erase(id);
    if (gnss_infos_.empty() && !is_previous_empty) {
      RemoveTableStreams("/sensor/other/gnss");
    }
    gnss_lock_.unlock();

    obstacle_lock_.lock();
    is_previous_empty = obstacle_infos_.empty();
    obstacle_infos_.erase(id);
    if (obstacle_infos_.empty() && !is_previous_empty) {
      RemoveTableStreams("/sensor/other/obstacle");
    }
    obstacle_lock_.unlock();
  }
  real_sensors_ = std::move(tmp_real_sensors);


  point_3d_t ego_position(0, 0, 0);
  point_3d_t ego_orientation(0, 0, 0);
  double display_velocity = 0;
  double display_acceleration = 0;

  if (ego_actor_ != nullptr) {
    if (!is_previous_ego_present_) {
      UpdateMetadataBuilder();
      is_previous_ego_present_ = true;
    }
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
  } else {
    if (is_previous_ego_present_) {
      UpdateMetadataBuilder();
      is_previous_ego_present_ = false;
    }
  }

  if (is_need_update_metadata_) {
    is_need_update_metadata_ = false;
    UpdateMetadata();
  }

  XVIZBuilder xviz_builder(metadata_ptr_);
  xviz_builder.UIPrimitive("/game/time")
    .Column("Game time/s", TreeTableColumn::DOUBLE, "second")
    .Column("Game frame", TreeTableColumn::INT32)
      .Row(0, {std::to_string(now_time), std::to_string(now_frame)});

  xviz_builder.Pose("/vehicle_pose")
    .MapOrigin(0, 0, 0)
    .Orientation(ego_orientation.get<0>(), ego_orientation.get<1>(), ego_orientation.get<2>())
    .Position(ego_position.get<0>(), ego_position.get<1>(), ego_position.get<2>())
    .Timestamp(now_time);

  if (IsStreamAllowTransmission("/vehicle/acceleration")) {
    xviz_builder.TimeSeries("/vehicle/acceleration")
      .Timestamp(now_time)
      .Value(display_acceleration)
      .Id("acceleration");
  }

  if (IsStreamAllowTransmission("/vehicle/velocity")) {
    xviz_builder.TimeSeries("/vehicle/velocity")
      .Timestamp(now_time)
      .Value(display_velocity)
      .Id("velocity");
  }


  std::vector<std::vector<double>> vehicle_vector;
  std::vector<uint32_t> vehicle_ids;
  std::vector<std::vector<double>> walker_vector;
  std::vector<uint32_t> walker_ids;

  for (const auto& actor_pair : actors_) {
    if (ego_actor_ != nullptr && ego_actor_->GetId() == actor_pair.first) {
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

  if (IsStreamAllowTransmission("/object/vehicles")) {
    for (auto i = 0; i < vehicle_vector.size(); i++) {
      xviz_builder.Primitive("/object/vehicles")
        .Polygon(std::move(vehicle_vector[i]))
        .ObjectId(std::to_string(vehicle_ids[i]))
        .Classes({"vehicle"});
    }
  }
  
  if (IsStreamAllowTransmission("/object/walkers")) {
    for (auto i = 0; i < walker_vector.size(); i++) {
      xviz_builder.Primitive("/object/walkers")
        .Polygon(std::move(walker_vector[i]))
        .ObjectId(std::to_string(walker_ids[i]))
        .Classes({"walker"});
    }
  }

  std::vector<uint32_t> to_delete_image_ids;
  std::unordered_map<uint32_t, std::string> tmp_non_experimental_server_images_;
  image_data_lock_.lock();
  for (const auto [camera_id, is_received] : is_image_received_) {
    if (real_dummy_sensors_relation_.find(camera_id) ==
        real_dummy_sensors_relation_.end()) {
      to_delete_image_ids.push_back(camera_id);
      continue;
    }
    if (!is_received) {
      continue;
    }
    is_image_received_[camera_id] = false;
    if (is_experimental_server_enabled_) {
      last_received_images_[camera_id] = std::move(image_data_queues_[camera_id].GetData());
    } else {
      if (IsStreamAllowTransmission(camera_streams_[camera_id])) {
        tmp_non_experimental_server_images_[camera_id] = std::move(image_data_queues_[camera_id].encoded_str_);
      }
    }
  }

  for (auto image_id : to_delete_image_ids) {
    image_data_queues_.erase(image_id);
    last_received_images_.erase(image_id);
  }
  image_data_lock_.unlock();

  if (is_experimental_server_enabled_) {
    for (const auto& [camera_id, image_str] : last_received_images_) {
      if (IsStreamAllowTransmission(camera_streams_[camera_id])) {
        xviz_builder.Primitive(camera_streams_[camera_id]).Image(image_str);
      }
    }
  } else {
    for (auto& [camera_id, image_str] : tmp_non_experimental_server_images_) {
        xviz_builder.Primitive(camera_streams_[camera_id]).Image(std::move(image_str));
    }
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


  if (!points.empty() && IsStreamAllowTransmission("/lidar/points")) {
    point_cloud_builder.Points(std::move(points));
  }

  collision_lock_.lock();
  if (!collision_events_.empty() && IsStreamAllowTransmission("/sensor/other/collision")) {
    XVIZUIPrimitiveBuilder& ui_primitive_builder = xviz_builder.UIPrimitive("/sensor/other/collision");
    ui_primitive_builder
      .Column("Self actor", TreeTableColumn::STRING)
      .Column("Other actor", TreeTableColumn::STRING)
      .Column("Timestamp", TreeTableColumn::DOUBLE)
      .Column("Frame", TreeTableColumn::INT32);
    size_t row_id = 0u;
    for (const auto& [id, event] : collision_events_) {
      ui_primitive_builder
        .Row(row_id, {event.GetSelfActorName(), event.GetOtherActorName(), 
                      std::to_string(event.GetLastHitTimestamp()), std::to_string(event.GetLastHitFrame())});
      row_id++;
    }
  }
  collision_lock_.unlock();

  gnss_lock_.lock();
  if (!gnss_infos_.empty() && IsStreamAllowTransmission("/sensor/other/gnss")) {
    XVIZUIPrimitiveBuilder& ui_primitive_builder = xviz_builder.UIPrimitive("/sensor/other/gnss");
    ui_primitive_builder
      .Column("Actor", TreeTableColumn::STRING)
      .Column("Lat", TreeTableColumn::DOUBLE)
      .Column("Lon", TreeTableColumn::DOUBLE)
      .Column("Alt", TreeTableColumn::DOUBLE)
      .Column("Timestamp", TreeTableColumn::DOUBLE);
    size_t row_id = 0u;
    for (const auto& [id, info] : gnss_infos_) {
      auto gnss_pos = info.GetPositions();
      ui_primitive_builder
        .Row(row_id, {info.GetSelfActorName(), std::to_string(gnss_pos[0]), std::to_string(gnss_pos[1]),
                      std::to_string(gnss_pos[2]), std::to_string(info.GetTimestamp())});
      row_id++;
    }
  }
  gnss_lock_.unlock();

  obstacle_lock_.lock();
  if (!obstacle_infos_.empty() && IsStreamAllowTransmission("/sensor/other/obstacle")) {
    XVIZUIPrimitiveBuilder& ui_primitive_builder = xviz_builder.UIPrimitive("/sensor/other/obstacle");
    ui_primitive_builder
      .Column("Self actor", TreeTableColumn::STRING)
      .Column("Other actor", TreeTableColumn::STRING)
      .Column("Detection distance", TreeTableColumn::DOUBLE)
      .Column("Timestamp", TreeTableColumn::DOUBLE)
      .Column("Frame", TreeTableColumn::INT32);
    size_t row_id = 0u;
    size_t current_frame = world_snapshots.GetFrame();
    for (auto& [id, info] : obstacle_infos_) {
      ui_primitive_builder
        .Row(row_id, {info.GetSelfActorName(), info.GetOtherActorName(), 
              std::to_string(info.GetDistance()), 
              std::to_string(info.GetTimestamp()), std::to_string(info.GetFrame())});
      row_id++;
    }
  }
  obstacle_lock_.unlock();

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


void CarlaProxy::HandleSensorData(uint32_t id, double rotation_frequency, 
  const std::string& type_id, const std::string& parent_name, carla::SharedPtr<carla::sensor::SensorData> data) {
  if (data == nullptr) {
    return;
  }
  
  // image
  if (Utils::IsStartWith(type_id, "sensor.camera")) {
    auto image_data =
      boost::dynamic_pointer_cast<carla::sensor::data::Image>(data);
    if (image_data == nullptr) {
      LOG_WARNING("Data received from %s is not image data", type_id.c_str());
      return;
    }
    utils::Image encoded_image;
    if (type_id == "sensor.camera.rgb") {
      encoded_image = GetEncodedRGBImage(*image_data);
    } else if (type_id == "sensor.camera.depth") {
      encoded_image = GetEncodedDepthImage(*image_data);
    } else if (type_id == "sensor.camera.semantic_segmentation") {
      encoded_image = GetEncodedLabelImage(*image_data);
    } else {
      LOG_ERROR("Unknown camera type: %s", type_id.c_str());
    }
    image_data_lock_.lock();
    if (image_data_queues_.find(id) != image_data_queues_.end()) {
      if (encoded_image.GetTimestamp() > image_data_queues_[id].GetTimestamp()) {
        is_image_received_[id] = true;
        image_data_queues_[id] = std::move(encoded_image);
      }
    } else {
      is_image_received_[id] = true;
      image_data_queues_[id] = std::move(encoded_image);
    }
    image_data_lock_.unlock();
    return;
  }

  // lidar
  if (type_id == "sensor.lidar.ray_cast") {
    auto lidar_data = boost::dynamic_pointer_cast<
      carla::sensor::data::LidarMeasurement>(data);
    if (lidar_data == nullptr) {
      LOG_WARNING("Data received from %s is not lidar data", type_id.c_str());
      return;
    }
    auto point_cloud = GetPointCloud(*(lidar_data));
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

  // collision
  if (type_id == "sensor.other.collision") {
    auto collision_data = boost::dynamic_pointer_cast<
      carla::sensor::data::CollisionEvent>(data);
    if (collision_data == nullptr) {
      LOG_WARNING("Data received from %s is not collision data", type_id.c_str());
      return;
    }
    auto collision_event = GetCollision(*collision_data, parent_name);
    collision_lock_.lock();
    collision_events_[id] = std::move(collision_event);
    collision_lock_.unlock();
    return;
  }

  // gnss event
  if (type_id == "sensor.other.gnss") {
    auto gnss_data = boost::dynamic_pointer_cast<
      carla::sensor::data::GnssEvent>(data);
    if (gnss_data == nullptr) {
      LOG_WARNING("Data received from %s is not gnss data", type_id.c_str());
      return;
    }
    auto gnss_info = GetGNSSInfo(*gnss_data, parent_name);
    gnss_lock_.lock();
    gnss_infos_[id] = std::move(gnss_info);
    gnss_lock_.unlock();
    return;
  }

  // obstacle info
  if (type_id == "sensor.other.obstacle") {
    auto obstacle_data = boost::dynamic_pointer_cast<
      carla::sensor::data::ObstacleDetectionEvent>(data);
    if (obstacle_data == nullptr) {
      LOG_WARNING("Data received from %s is not obstacle data", type_id.c_str());
      return;
    }
    auto obstacle_info = GetObstacleInfo(*obstacle_data, parent_name);
    obstacle_lock_.lock();
    obstacle_infos_[id] = std::move(obstacle_info);
    obstacle_lock_.unlock();
    return;
  }

  LOG_WARNING("Receive unhandled data %s", type_id.c_str());
}

std::pair<std::string, boost::shared_ptr<carla::client::Sensor>> CarlaProxy::CreateDummySensor(
    boost::shared_ptr<carla::client::Sensor> real_sensor) {
  auto real_sensor_attribute = real_sensor->GetAttributes();
  auto type_id = real_sensor->GetTypeId();
  auto blueprint_lib = world_ptr_->GetBlueprintLibrary();
  auto blueprint = (*(blueprint_lib->Filter(type_id)))[0];

  for (const auto& attribute : real_sensor_attribute) {
    blueprint.SetAttribute(attribute.GetId(), attribute.GetValue());
  }

  auto parent = real_sensor->GetParent();
  std::string parent_name;
  auto parent_transform = carla::geom::Transform();
  if (parent == nullptr) {
    LOG_WARNING("Real sensor with id %ud has no attached actor, "
                "did you attach it to an actor or successfully desotry it last time?",
                real_sensor->GetId());
    parent_name = "null";
  } else {
    parent_transform = parent->GetTransform();
    parent_name = parent->GetTypeId() + " " + std::to_string(parent->GetId());
  }
  auto sensor_transform = real_sensor->GetTransform();
  auto relative_transform =
      GetRelativeTransform(sensor_transform, parent_transform);

  auto dummy_sensor = boost::static_pointer_cast<carla::client::Sensor>(
      world_ptr_->SpawnActor(blueprint, relative_transform, parent.get()));
  return {parent_name, dummy_sensor};
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

utils::Image CarlaProxy::GetEncodedRGBImage(
    carla::sensor::data::Image& image) {
  size_t pixel_size = image.size();
  auto color_ptr = (unsigned char*) image.data();
  std::vector<unsigned char> pixel_data(pixel_size * 3u);
  for (auto i = 0; i < pixel_size; i++) {
    pixel_data[3*i] = color_ptr[4*i + 2u];
    pixel_data[3*i + 1u] = color_ptr[4*i + 1u];
    pixel_data[3*i + 2u] = color_ptr[4*i];
  }
  std::vector<unsigned char> image_data;
  unsigned int error = lodepng::encode(image_data, pixel_data, image.GetWidth(),
                                       image.GetHeight(), LodePNGColorType::LCT_RGB);
  if (error) {
    LOG_ERROR("Encoding png error");
  }
  std::string data_str(image_data.begin(), image_data.end());
  return utils::Image(std::move(data_str), image.GetWidth(), image.GetHeight(), image.GetTimestamp());
}

utils::Image CarlaProxy::GetEncodedDepthImage(
    const carla::sensor::data::Image& image) {
  std::vector<unsigned char> pixel_data;
  for (const auto& p : image) {
    auto depth = (unsigned char)((p.r + p.g * 256.0 + p.b * 256.0 * 256.0) / (256.0 * 256.0 * 256.0 - 1.0) * 255.0);
    pixel_data.emplace_back(depth);
    pixel_data.emplace_back(depth);
    pixel_data.emplace_back(depth);
    pixel_data.emplace_back(255);
  }
  std::vector<unsigned char> image_data;
  unsigned int error = lodepng::encode(image_data, pixel_data, image.GetWidth(),
                                       image.GetHeight());
  if (error) {
    LOG_ERROR("Encoding png error");
  }
  std::string data_str(image_data.begin(), image_data.end());
  return utils::Image(std::move(data_str), image.GetWidth(), image.GetHeight(), image.GetTimestamp());
}

utils::Image CarlaProxy::GetEncodedLabelImage(
    const carla::sensor::data::Image& image) {
  std::vector<unsigned char> pixel_data;
  for (const auto& p : image) {
    if (label_color_map.find(p.r) == label_color_map.end()) {
      LOG_WARNING("Unknown tag: %u", p.r);
      auto& color = label_color_map[0];
      pixel_data.emplace_back(color[0]);
      pixel_data.emplace_back(color[1]);
      pixel_data.emplace_back(color[2]);
      pixel_data.emplace_back(255);
    } else {
      auto& color = label_color_map[p.r];
      pixel_data.emplace_back(color[0]);
      pixel_data.emplace_back(color[1]);
      pixel_data.emplace_back(color[2]);
      pixel_data.emplace_back(255);
    }
  }
  std::vector<unsigned char> image_data;
  unsigned int error = lodepng::encode(image_data, pixel_data, image.GetWidth(),
                                       image.GetHeight());
  if (error) {
    LOG_ERROR("Encoding png error");
  }
  std::string data_str(image_data.begin(), image_data.end());
  return utils::Image(std::move(data_str), image.GetWidth(), image.GetHeight(), image.GetTimestamp());
}

CollisionEvent CarlaProxy::GetCollision(const carla::sensor::data::CollisionEvent& collision_event,
  const std::string& parent_name) {
  auto other_actor = collision_event.GetOtherActor();
  std::string other_actor_name;
  if (other_actor == nullptr) {
    LOG_WARNING("%s collides with null actor, it is wired.", parent_name.c_str());
    other_actor_name = "null";
  } else {
    other_actor_name = other_actor->GetTypeId() + " " + std::to_string(other_actor->GetId());
  }
  return CollisionEvent(0u, 0u, parent_name, other_actor_name, collision_event.GetTimestamp(), collision_event.GetFrame());
}

utils::GNSSInfo CarlaProxy::GetGNSSInfo(const carla::sensor::data::GnssEvent& gnss_event,
    const std::string& parent_name) {
  return GNSSInfo(parent_name, gnss_event.GetLatitude(), gnss_event.GetLongitude(),
                gnss_event.GetAltitude(), gnss_event.GetTimestamp());
}

utils::ObstacleInfo CarlaProxy::GetObstacleInfo(const carla::sensor::data::ObstacleDetectionEvent& obs_event,
  const std::string& parent_name) {
  auto other_actor = obs_event.GetOtherActor();
  std::string other_actor_name = "null";
  if (other_actor == nullptr) {
    LOG_WARNING("Other actor is null");
  } else {
    other_actor_name = other_actor->GetTypeId() + " " + std::to_string(other_actor->GetId());
  }
  return ObstacleInfo(parent_name, other_actor_name, obs_event.GetDistance(), obs_event.GetTimestamp(), obs_event.GetFrame());
}