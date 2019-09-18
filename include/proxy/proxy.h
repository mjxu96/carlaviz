/*
 * File: proxy.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:10:14 pm
 */

#ifndef MELLOCOLATE_PROXY_H_
#define MELLOCOLATE_PROXY_H_

#include "proxy/utils/def.h"
#include "proxy/utils/utils.h"
#include "proxy/xviz/xviz_builder.h"
#include "proxy/xviz/xviz_metadata_builder.h"

#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/ActorSnapshot.h"
#include "carla/client/Client.h"
#include "carla/client/Sensor.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"
#include "carla/client/World.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
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
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace mellocolate {

class Proxy {
 public:
  Proxy() = delete;
  Proxy(boost::shared_ptr<carla::client::Client> client_ptr);
  void Run();
  void AddClient(boost::asio::ip::tcp::socket socket);

 private:
  void Init();
  void Update(const std::string& data_str);

  // Carla related
  std::string GetMetaData();
  std::string GetUpdateData(
      const carla::client::WorldSnapshot& world_snapshots);
  void AddVehicle(XVIZPrimitiveBuider& xviz_primitive_builder,
                  boost::shared_ptr<carla::client::Vehicle> vehicle);
  void AddWalker(XVIZPrimitiveBuider& xviz_primitive_builder,
                 boost::shared_ptr<carla::client::Walker> walker);
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr};
  boost::shared_ptr<carla::client::Client> client_ptr_{nullptr};

  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Actor>> actors_;
  // Carla sensor related
  std::mutex sensor_data_queue_lock_;
  // std::unordered_map<uint32_t, std::queue<carla::sensor::SensorData>>
  // sensor_data_queues_{};
  std::unordered_map<uint32_t, std::vector<point_3d_t>> lidar_data_queues_{};
  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Sensor>>
      sensors_{};
  // Carla Lidar sensor data related
  std::vector<point_3d_t> GetPointCloud(
      const carla::sensor::data::LidarMeasurement& lidar_measurement);

  // Websocket related
  // boost::shared_ptr<
  //     boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>
  //     ws_ptr_{nullptr};
  std::mutex clients_addition_lock_;
  boost::unordered_set<boost::shared_ptr<
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>> ws_ptrs_;
};

}  // namespace mellocolate

#endif