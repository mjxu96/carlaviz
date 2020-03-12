/*
 * File: proxy.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:10:14 pm
 */

#ifndef MELLOCOLATE_PROXY_H_
#define MELLOCOLATE_PROXY_H_

#include "platform/utils/def.h"
#include "platform/utils/lodepng.h"
#include "platform/utils/utils.h"
#include "builder/xviz_builder.h"
#include "builder/metadata.h"
#include "builder/declarative_ui/ui_builder.h"
#include "builder/declarative_ui/video_builder.h"
#include "builder/declarative_ui/metric_builder.h"
#include "builder/declarative_ui/container_builder.h"

#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/ActorList.h"
#include "carla/client/ActorSnapshot.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/Sensor.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"
#include "carla/client/World.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/image/ImageView.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/sensor/data/CollisionEvent.h"
#include "carla/sensor/data/GnssEvent.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <ios>
#include <iostream>
#include <memory>
#include <mutex>
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
  void Clear();
  std::string GetMetadata();
  std::string GetMapString();
  xviz::XVIZBuilder GetUpdateData();
  void SetUpdateMetadataCallback(const std::function<void(const std::string&)>& func);

 private:
  xviz::XVIZBuilder GetUpdateData(
      const carla::client::WorldSnapshot& world_snapshots);
  std::mutex internal_update_builder_lock_{};
  xviz::XVIZBuilder internal_update_builder_{nullptr};
  std::string metadata_str_{};

  void UpdateMetadata();
  void AddTrafficLights(
      xviz::XVIZPrimitiveBuilder& xviz_primitive_builder,
      boost::shared_ptr<carla::client::TrafficLight> traffic_light);
  void AddTrafficLightAreas();

  // Metadata related
  xviz::XVIZMetadataBuilder GetBaseMetadataBuilder();
  void AddCameraStream(uint32_t camera_id, const std::string& stream_name="");
  void RemoveCameraStream(uint32_t camera_id);

  void AddEgoVehicleMetricStreams();
  void RemoveVehicleMetricStreams();

  void AddTableStreams(const std::string& sensor_type_name);
  void RemoveTableStreams(const std::string& sensor_type_name);

  void UpdateMetadataBuilder();

  std::unordered_map<uint32_t, std::vector<std::vector<double>>> traffic_lights_{};

  std::string carla_host_{"localhost"};
  uint16_t carla_port_{2000u};
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr};
  boost::shared_ptr<carla::client::Client> client_ptr_{nullptr};

  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Actor>> actors_;
  boost::shared_ptr<carla::client::Actor> ego_actor_{nullptr};
  int ego_id_{-1};

  std::shared_ptr<xviz::Metadata> metadata_ptr_{nullptr};

  // Metadata related
  xviz::XVIZMetadataBuilder metadata_builder_{};
  std::unordered_map<uint32_t, std::string> camera_streams_{};
  std::unordered_set<std::string> other_sensor_streams_{};
  std::function<void(const std::string&)> frontend_proxy_update_metadata_callback_;
  bool is_need_update_metadata_{false};

  // Carla sensor related
  std::mutex image_data_lock_;
  // bool is_image_received_{false};
  std::unordered_map<uint32_t, bool> is_image_received_{};
  std::unordered_map<uint32_t, utils::Image> image_data_queues_{};
  std::unordered_map<uint32_t, std::string> last_received_images_{};
  std::mutex lidar_data_lock_;
  std::unordered_map<uint32_t, std::deque<utils::PointCloud>>
      lidar_data_queues_{};
  std::unordered_map<uint32_t, uint32_t> real_dummy_sensors_relation_{};
  std::unordered_set<uint32_t> real_sensors_{};
  std::unordered_map<uint32_t, boost::shared_ptr<carla::client::Sensor>>
      dummy_sensors_{};
  std::unordered_set<uint32_t> recorded_dummy_sensor_ids_{};

  std::mutex collision_lock_;
  std::unordered_map<uint32_t, utils::CollisionEvent> collision_events_{};

  std::mutex gnss_lock_;
  std::unordered_map<uint32_t, utils::GNSSInfo> gnss_infos_{};

  // Carla sensor data related
  std::pair<std::string, boost::shared_ptr<carla::client::Sensor>>  CreateDummySensor(
      boost::shared_ptr<carla::client::Sensor> real_sensor);
  carla::geom::Transform GetRelativeTransform(
      const carla::geom::Transform& child,
      const carla::geom::Transform& parent);
  utils::Image GetEncodedRGBImage(const carla::sensor::data::Image& image);
  utils::Image GetEncodedDepthImage(const carla::sensor::data::Image& image);
  utils::Image GetEncodedLabelImage(const carla::sensor::data::Image& image);

  utils::PointCloud GetPointCloud(
      const carla::sensor::data::LidarMeasurement& lidar_measurement);

  utils::CollisionEvent GetCollision(const carla::sensor::data::CollisionEvent& collision_event,
    const std::string& parent_name);

  utils::GNSSInfo GetGNSSInfo(const carla::sensor::data::GnssEvent& gnss_event,
    const std::string& parent_name);

  // Websocket related
  std::mutex clients_addition_lock_;
  boost::unordered_set<boost::shared_ptr<
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>>
      ws_ptrs_;
};

}  // namespace mellocolate

#endif