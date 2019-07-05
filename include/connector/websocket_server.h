#ifndef WEBSOCKET_SERVER_H_
#define WEBSOCKET_SERVER_H_

#include "connector/utils/package.h"
#include "connector/utils/json.hpp"
#include "connector/xviz/xviz_metadata_builder.h"
#include "connector/xviz/xviz_builder.h"

#include "carla/client/Vehicle.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/client/Sensor.h"

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
#include <unordered_set>

namespace mellocolate {


class WebsocketServer {
public:
  WebsocketServer(std::string map_json) : map_json_(std::move(map_json)) {}
  void Init(boost::shared_ptr<mellocolate::utils::Package> package_ptr, 
    boost::shared_ptr<std::mutex> package_mutex, 
    std::string host="127.0.0.1", uint16_t port=8081u);
  void Run();

private:
  void DoSession(boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket);

  std::string GetInitMetaDataJson();
  std::string GetLiveDataJson();

  std::string host_{"127.0.0.1"};
  uint16_t port_{8081u};
  boost::shared_ptr<mellocolate::utils::Package> package_ptr_{nullptr};
  boost::shared_ptr<std::mutex> package_mutex_{nullptr};
  boost::shared_ptr<std::mutex> internal_point_cloud_mutex_{boost::make_shared<std::mutex>()};
  boost::shared_ptr<std::mutex> internal_lidar_set_mutex_{boost::make_shared<std::mutex>()};
  std::vector<std::thread> threads_{};

  // Sensor data
  //void LidarDataCallback(boost::shared_ptr<carla::sensor::SensorData> lidar_measurement);
  void LidarDataCallback(const carla::sensor::data::LidarMeasurement& lidar_measurement,
    const carla::geom::Location& location);
  std::vector<point_3d_t> points_;

  std::unordered_set<uint32_t> registered_sensor_id_;

  // TODO remove tmp
  double tmp_pos_x{0};
  double tmp_pos_y{0};
  
  std::string map_json_{};
};

} // namespace mellocolate
#endif