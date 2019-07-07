/*
 * File: connector_tick.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 12:58:35 pm
 */

#ifndef MELLOCOLATE_CONNECTOR_TICK_H_
#define MELLOCOLATE_CONNECTOR_TICK_H_

#include "connector/utils/def.h"
#include "carla/client/World.h"

#include <boost/shared_ptr.hpp>

namespace mellocolate {

class Connector {
public:
  Connector(SharedPtr<carla::client::World> world_ptr);
  void Run();
private:
  SharedPtr<carla::client::World> world_ptr_{nullptr};
};

} // namespace mellocolate


#endif