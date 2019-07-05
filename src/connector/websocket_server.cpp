#include "connector/websocket_server.h"

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace mellocolate {

std::pair<double, double> AfterRotate(double x, double y, double yaw) {
  return {std::cos(yaw)*x - std::sin(yaw)*y, std::sin(yaw)*x + std::cos(yaw)*y};
}

std::string ReadGeoJsonFromFile(std::string file_name) {
  std::ifstream t(file_name);
  std::stringstream buffer;
  buffer << t.rdbuf();
  return buffer.str();
}


void WebsocketServer::Init(boost::shared_ptr<mellocolate::utils::Package> package_ptr, 
  boost::shared_ptr<std::mutex> package_mutex,
  std::string host, uint16_t port) {
  package_ptr_ = std::move(package_ptr);
  package_mutex_ = std::move(package_mutex);
  host_ = std::move(host);
  port_ = port;
}

void WebsocketServer::Run() {
  try {
    if (host_.size() == 0) {
      std::cerr << "[Websocket Server Error] Please indicate ip address" << std::endl;
      return;
    }

    std::cout << host_ << std::endl;
    const auto address =  boost::asio::ip::make_address(host_);
    const auto port = port_;

    boost::asio::io_context ioc{1};
    boost::asio::ip::tcp::acceptor accetor{ioc, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)};
    //{address, port}};

    std::cout << "[Websocket Server Log] Start listening to " << address << ":" << port << std::endl;

    while (true) {
      boost::asio::ip::tcp::socket socket{ioc};
      accetor.accept(socket);
      std::cout << "[Websocket Server Log] Client connected" << std::endl;
      std::thread t = std::thread{std::bind(
        &WebsocketServer::DoSession, this,
        std::move(socket))};
      t.detach();
      threads_.push_back(std::move(t));
    }

  } catch (const std::exception& e) {
    std::cerr << "[Websocket Server Error] Error: " << e.what() << std::endl;
    return;
  }
}

void WebsocketServer::DoSession(boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket) {

  try {
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws{std::move(socket)};

    ws.accept();
    boost::beast::multi_buffer init_buffer;
    std::string init_meda_data = GetInitMetaDataJson();
    boost::beast::ostream(init_buffer) << init_meda_data;
    ws.write(init_buffer.data());
    for(;;) {
      boost::beast::multi_buffer buffer;
   
      std::string update_data = GetLiveDataJson();
      boost::beast::ostream(buffer) << update_data;
      ws.write(buffer.data());
      std::this_thread::sleep_for(40ms);
    }
  } catch(boost::system::system_error const& se) {
    if(se.code() != boost::beast::websocket::error::closed) {
      std::cerr << "Error: " << se.code().message() << std::endl;
    } else {
      std::cout << "Session closed" << std::endl;
    }
  } catch(std::exception const& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

}

std::string WebsocketServer::GetInitMetaDataJson() {
  XVIZMetaDataBuilder xviz_metadata_builder;
  xviz_metadata_builder
    .SetMap(map_json_)
    .AddStream(metadata::Stream("/vehicle_pose")
      .AddCategory("pose"))
    .AddStream(metadata::Stream("/object/shape")
      .AddCategory("primitive")
      .AddCoordinate("IDENTITY")
      .AddStreamStyle(metadata::StreamStyle()
        .AddExtruded(true)
        .AddFillColor("#fb0")
        .AddHeight(1.5))
      .AddType("polygon"))
    .AddStream(metadata::Stream("/lidar/points")
      .AddCategory("primitive")
      .AddCoordinate("IDENTITY")
      .AddType("points")
      .AddStreamStyle(metadata::StreamStyle()
        .AddPointCloudMode("elevation")
        .AddRadiusPixels(3.0)));
  return xviz_metadata_builder.GetMetaData();
}

std::string WebsocketServer::GetLiveDataJson() {
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

  package_mutex_->lock();
  auto actor_list = package_ptr_->GetActorListPtr();
  int i = 0;
  double of = 2.0;
  std::vector<std::pair<double, double>> offset = {{-of, -of}, {-of, of}, {of, of}, {of, -of}};
  for (const auto& actor : *actor_list) {
    // Lidar sensor
    if (actor->GetTypeId() == "sensor.lidar.ray_cast") {
      uint32_t id = actor->GetId();
      internal_lidar_set_mutex_->lock();
      if (registered_sensor_id_.find(id) == registered_sensor_id_.end()) {
        registered_sensor_id_.insert(id);
        internal_lidar_set_mutex_->unlock();
        (boost::static_pointer_cast<carla::client::Sensor>(actor))->Listen(
          [=] (carla::SharedPtr<carla::sensor::SensorData> data) {
            if (actor->IsAlive()) {
              auto cast_data = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
              this->LidarDataCallback(*cast_data, actor->GetLocation());
            } else {
              (boost::static_pointer_cast<carla::client::Sensor>(actor))->Stop();
              internal_lidar_set_mutex_->lock();
              registered_sensor_id_.erase(id);
              internal_lidar_set_mutex_->unlock();
            }
          }
        );
      } else {
        internal_lidar_set_mutex_->unlock();
      }
    }

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
  package_mutex_->unlock();

  xviz_builder
    .AddPrimitive(xviz_primitive_builder);

  // Lidar sensor
  internal_point_cloud_mutex_->lock();
  xviz_builder
    .AddPrimitive(XVIZPrimitiveBuider("/lidar/points")
      .AddPoints(XVIZPrimitivePointBuilder(points_)));
  internal_point_cloud_mutex_->unlock();

  return xviz_builder.GetData();
}

void WebsocketServer::LidarDataCallback(const carla::sensor::data::LidarMeasurement& lidar_measurement,
  const carla::geom::Location& location) {
  internal_point_cloud_mutex_->lock();
  points_.clear();
  for (const auto& point : lidar_measurement) {
    points_.emplace_back(point.x + location.x, - point.y - location.y, point.z + location.z);
  }
  internal_point_cloud_mutex_->unlock();
}

} // namespace mellocolate
