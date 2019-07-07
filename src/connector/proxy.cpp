/*
 * File: proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:11:52 pm
 */

#include "connector/proxy.h"

using namespace mellocolate;
// For readable seconds
using namespace std::chrono_literals;
using namespace std::string_literals;

// For websocket
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

std::pair<double, double> AfterRotate(double x, double y, double yaw) {
  return {std::cos(yaw)*x - std::sin(yaw)*y, std::sin(yaw)*x + std::cos(yaw)*y};
}

Proxy::Proxy(std::string carla_host, uint16_t carla_port, uint16_t ws_port)
    : carla_host_(std::move(carla_host)),
      carla_port_(carla_port),
      ws_port_(ws_port) {}

void Proxy::Run() { 
  Init(); 
  while (true) {
    try {
      Update();
      world_ptr_->WaitForTick(200ms);
    } catch (const std::exception& e) {
      LOG_ERROR("%s", e.what());
    }
  }
}

void Proxy::Init() {
  try {
    // Connect to Carla server
    carla::client::Client client(carla_host_, carla_port_);
    client.SetTimeout(10s);
    LOG_INFO("Connecting to Carla Server on %s:%u...", carla_host_.c_str(),
             carla_port_);
    world_ptr_ = boost::make_shared<carla::client::World>(client.GetWorld());
    LOG_INFO("Connected to Carla Server");

    ws_accept_thread_ = std::thread(&Proxy::Accept, this);
    ws_accept_thread_.detach();

  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void Proxy::Update() {

  boost::beast::multi_buffer buffer;
  
  //std::string update_data = "Hello";
  boost::beast::ostream(buffer) << GetUpdateData();

  // Update to all clients
  std::vector<std::shared_ptr<websocket::stream<tcp::socket>>> to_delete_ws;
  std::lock_guard<std::mutex> lock_gurad(ws_lock_);
  for (const auto& ws_ptr : ws_set_) {
    try {
      ws_ptr->write(buffer.data());
    } catch (const std::exception& e) {
      LOG_ERROR("%s", e.what());
      to_delete_ws.push_back(ws_ptr);
    }
  }
  for (const auto& ws_ptr : to_delete_ws) {
    if (ws_set_.find(ws_ptr) != ws_set_.end()) {
      ws_set_.erase(ws_ptr);
    }
  }
}

std::string Proxy::GetMetaData() {
  std::string map_geojson = utils::XodrGeojsonConverter::GetGeoJsonFromCarlaMap(world_ptr_->GetMap());
  XVIZMetaDataBuilder xviz_metadata_builder;
  xviz_metadata_builder
    .SetMap(map_geojson)
    .AddStream(metadata::Stream("/vehicle_pose")
      .AddCategory("pose"))
    .AddStream(metadata::Stream("/object/shape")
      .AddCategory("primitive")
      .AddCoordinate("IDENTITY")
      .AddStreamStyle(metadata::StreamStyle()
        .AddExtruded(true)
        .AddFillColor("#fb0")
        .AddHeight(1.5))
      .AddType("polygon"));
  return xviz_metadata_builder.GetMetaData();
}

std::string Proxy::GetUpdateData() {
  std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  double now_time = now.time_since_epoch().count() / 1e9;

  XVIZBuilder xviz_builder;
  xviz_builder
    .AddTimestamp(now_time)
    .AddPose(XVIZPoseBuilder("/vehicle_pose")
      .AddMapOrigin(point_3d_t(0, 0, 0))
      .AddOrientation(point_3d_t(0, 0, 0))
      .AddPosition(point_3d_t(0, 0, 0))
      .AddTimestamp(now_time));
  XVIZPrimitiveBuider xviz_primitive_builder("/object/shape");

  auto actor_list = world_ptr_->GetActors();
  int i = 0;
  double of = 2.0;
  std::vector<std::pair<double, double>> offset = {{-of, -of}, {-of, of}, {of, of}, {of, -of}};
  for (const auto& actor : *actor_list) {
    if (actor->GetTypeId().substr(0, 2) != "ve") {
      continue;
    }
    auto bounding_box = (boost::static_pointer_cast<carla::client::Vehicle>(actor))->GetBoundingBox();
    double x_off = bounding_box.extent.x;
    double y_off = bounding_box.extent.y;
    double yaw = actor->GetTransform().rotation.yaw / 180.0 * M_PI;
    offset = {AfterRotate(-x_off, -y_off, yaw), AfterRotate(-x_off, y_off, yaw),
              AfterRotate(x_off, y_off, yaw), AfterRotate(x_off, -y_off, yaw)};
    double x = actor->GetLocation().x;
    double y = actor->GetLocation().y;
    double z = actor->GetLocation().z;
    std::vector<point_3d_t> vertices;
    for (int j = 0; j < offset.size(); j++) {
      vertices.emplace_back(x + offset[j].first, -(y + offset[j].second), z);
    }
    xviz_primitive_builder
        .AddPolygon(XVIZPrimitivePolygonBuilder(vertices)
          .AddId(actor->GetTypeId() + std::to_string(actor->GetId())));
  }

  xviz_builder
    .AddPrimitive(xviz_primitive_builder);

  return xviz_builder.GetData();
}

void Proxy::Accept() {
  LOG_INFO("Connecting to frontend client. Listening to port %u....", ws_port_);
  try {
    boost::asio::io_context ioc{1};

    tcp::acceptor acceptor{ioc, tcp::endpoint(tcp::v4(), ws_port_)};
    for (;;) {
      tcp::socket socket{ioc};

      acceptor.accept(socket);

      AddClient(std::move(socket));
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void Proxy::AddClient(tcp::socket socket) {
  std::shared_ptr<websocket::stream<tcp::socket>> ws_ptr =
      std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
  try {
    std::lock_guard<std::mutex> lock_gurad(ws_lock_);
    ws_ptr->accept();
    LOG_INFO("Client connected.");
    boost::beast::multi_buffer buffer;
    boost::beast::ostream(buffer) << GetMetaData();
    ws_ptr->write(buffer.data());
    ws_set_.insert(ws_ptr);
  } catch (std::exception const& e) {
    std::lock_guard<std::mutex> lock_gurad(ws_lock_);
    if (ws_set_.find(ws_ptr) != ws_set_.end()) {
      ws_set_.erase(ws_ptr);
    }
    LOG_ERROR("%s", e.what());
  }
}

int main() {
  Proxy proxy;
  proxy.Run();
  return 0;
}