/*
 * File: proxy.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:10:14 pm
 */

#ifndef MELLOCOLATE_PROXY_H_
#define MELLOCOLATE_PROXY_H_

#include "platform/utils/base64.h"
#include "platform/utils/def.h"
#include "platform/utils/lodepng.h"
#include "platform/utils/utils.h"
#include "platform/xviz/xviz_builder.h"
#include "platform/xviz/xviz_metadata_builder.h"

#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/ActorList.h"
#include "carla/client/ActorSnapshot.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/Sensor.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"
#include "carla/client/World.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/image/ImageView.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/LidarMeasurement.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <ios>
#include <iostream>
#include <memory>
#include <mutex>
#include <deque>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace mellocolate {

class CarlaProxy {
 public:
  CarlaProxy() = default;
  CarlaProxy(const std::string& carla_host, uint16_t carla_port);
  CarlaProxy(boost::shared_ptr<carla::client::Client> client_ptr);
  void Init();
  std::string GetMetaData();
  XVIZBuilder GetUpdateData(
      const carla::client::WorldSnapshot& world_snapshots);
  XVIZBuilder GetUpdateData();
  void Run();
  void AddClient(boost::asio::ip::tcp::socket socket);

 private:
  void Update(const std::string& data_str);

  // Carla related
  // std::string GetUpdateData(
      // const carla::client::WorldSnapshot& world_snapshots);
  void AddVehicle(XVIZPrimitiveBuider& xviz_primitive_builder,
                  boost::shared_ptr<carla::client::Vehicle> vehicle);
  void AddWalker(XVIZPrimitiveBuider& xviz_primitive_builder,
                 boost::shared_ptr<carla::client::Walker> walker);

  std::string carla_host_{"localhost"};
  uint16_t carla_port_{2000u};
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr};
  boost::shared_ptr<carla::client::Client> client_ptr_{nullptr};

  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Actor>> actors_;

  // Carla sensor related
  std::mutex image_data_lock_;
  bool is_image_received_{false};
  std::unordered_map<uint32_t, utils::Image> image_data_queues_{};
  std::mutex lidar_data_lock_;
  std::unordered_map<uint32_t,
                     std::deque<utils::PointCloud>>
      lidar_data_queues_{};
  std::unordered_map<uint32_t, uint32_t> real_dummy_sensors_relation_{};
  std::unordered_set<uint32_t> real_sensors_{};
  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Sensor>>
      dummy_sensors_{};

  // Carla sensor data related
  boost::shared_ptr<carla::client::Sensor> CreateDummySensor(
      boost::shared_ptr<carla::client::Sensor> real_sensor);
  carla::geom::Transform GetRelativeTransform(
      const carla::geom::Transform& child,
      const carla::geom::Transform& parent);
  utils::Image GetEncodedImage(const carla::sensor::data::Image& image);
  utils::PointCloud GetPointCloud(
      const carla::sensor::data::LidarMeasurement& lidar_measurement);

  // Websocket related
  std::mutex clients_addition_lock_;
  boost::unordered_set<boost::shared_ptr<
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>>
      ws_ptrs_;
};

}  // namespace mellocolate

#endif