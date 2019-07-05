#ifndef CONNECTOR_PACKAGE_H_
#define CONNECTOR_PACKAGE_H_

#include "carla/road/MapData.h"
#include "carla/road/Map.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"

#include "carla/sensor/data/LidarMeasurement.h"

#include "carla/opendrive/OpenDriveParser.h"

#include "connector/utils/def.h"

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <vector>

namespace mellocolate {
namespace utils {

class Package {
public:
  Package(boost::shared_ptr<carla::client::World> world_ptr);
  void Update();
  void UpdatePointCloud(const boost::shared_ptr<carla::sensor::data::LidarMeasurement>& lidar_measurement);
  boost::shared_ptr<carla::client::ActorList> GetActorListPtr() const;
  void TmpOutput();

private:

  void Clear();
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr};

  boost::shared_ptr<carla::client::ActorList> actor_list_ptr_{nullptr};
  boost::shared_ptr<carla::road::Map> map_detail_{nullptr};
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};
  boost::shared_ptr<std::vector<point_3d_t>> points_{boost::make_shared<std::vector<point_3d_t>>()};
};
  

  
} // namespace utils
} // namespace mellocolate



#endif