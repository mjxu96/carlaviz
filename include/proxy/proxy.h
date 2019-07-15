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

#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/Vehicle.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/client/Sensor.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/client/ActorSnapshot.h"

#include <boost/shared_ptr.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>


#include <cstdlib>
#include <functional>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>
#include <cmath>
#include <memory>
#include <unordered_set>
#include <queue>

namespace mellocolate {

class Proxy {
public:
  Proxy() = default;
  Proxy(std::string carla_host, uint16_t carla_port, uint16_t ws_port);
  void Run();
  void AddClient(boost::asio::ip::tcp::socket socket);
  void TmpSetWorldPtr(boost::shared_ptr<carla::client::World> world_ptr) {
    world_ptr_ = world_ptr;
  }

private:
  void Init();
  void Update();

  // Carla related
  std::string GetMetaData();
  std::string GetUpdateData();
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr}; 
  std::string carla_host_{"localhost"};
  uint16_t carla_port_{2000u};
  // Carla sensor related
  std::mutex sensor_data_queue_lock_;
  //std::unordered_map<uint32_t, std::queue<carla::sensor::SensorData>> sensor_data_queues_{};
  std::unordered_map<uint32_t, std::vector<point_3d_t>> lidar_data_queues_{};
  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Sensor>> sensors_{};
  // Carla Lidar sensor data related
  std::vector<point_3d_t> GetPointCloud(const carla::sensor::data::LidarMeasurement& lidar_measurement);

  // Websocket related
  void Accept();
  uint16_t ws_port_{8081u};
  std::thread ws_accept_thread_;
  std::mutex ws_lock_;
  std::unordered_set<std::shared_ptr<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>> ws_set_{};
};

} // namespace mellocolate


#endif