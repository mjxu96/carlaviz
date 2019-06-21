#ifndef CONNECTOR_PACKAGE_H_
#define CONNECTOR_PACKAGE_H_

#include "carla/road/MapData.h"
#include "carla/road/Map.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"

#include "carla/opendrive/OpenDriveParser.h"

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <vector>

namespace rothberg {
namespace utils {

class Package {
public:
  Package(boost::shared_ptr<carla::client::World> world_ptr);
  void Update();
  void TmpOutput();
  std::vector<char> ToWebSocketData();

private:

  void Clear();
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr};

  boost::shared_ptr<carla::client::ActorList> actor_list_ptr_{nullptr};
  boost::shared_ptr<carla::road::Map> map_detail_{nullptr};
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};

};
  

  
} // namespace utils
} // namespace rothberg



#endif