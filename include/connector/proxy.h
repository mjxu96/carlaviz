/*
 * File: proxy.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:10:14 pm
 */

#ifndef MELLOCOLATE_PROXY_H_
#define MELLOCOLATE_PROXY_H_

#include "connector/utils/def.h"
#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/TimeoutException.h"

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

class Proxy {
public:
  Proxy() = default;
  Proxy(std::string carla_host, uint16_t carla_port, uint16_t ws_port);
  void Run();

private:
  void Init();
  void Update();

  // Carla client and world ptr
  boost::shared_ptr<carla::client::World> world_ptr_{nullptr}; 

  // Carla connection variables
  std::string carla_host_{"localhost"};
  uint16_t carla_port_{2000u};

  // Websocket variables
  uint16_t ws_port_{8081u};
  std::thread ws_accept_thread_;
};

} // namespace mellocolate


#endif