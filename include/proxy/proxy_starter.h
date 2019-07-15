/*
 * File: proxy_starter.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 14th July 2019 6:42:08 pm
 */

#include "proxy/utils/def.h"
#include "proxy/utils/utils.h"

#include "proxy/proxy.h"

#include "carla/client/Client.h"

#include <boost/shared_ptr.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <string>
#include <thread>

#ifndef MELLOCOLATE_PROXY_STARTER_H_
#define MELLOCOLATE_PROXY_STARTER_H_

namespace mellocolate {

class ProxyStarter {
public:
  ProxyStarter() = default;
  ProxyStarter(std::string carla_ip, uint16_t carla_port, std::string ws_ip, uint16_t ws_port);
  void Run();

private:
  void Init();
  void Accpet();
  void AddClient(boost::asio::ip::tcp::socket socket);

  boost::shared_ptr<carla::client::Client> carla_client_ptr_{nullptr};

  std::string carla_ip_{"localhost"};
  uint16_t carla_port_{2000u};

  std::string ws_ip_{"0.0.0.0"};
  uint16_t ws_port_{8081u};
};
  
} // namespace mellocolate



#endif