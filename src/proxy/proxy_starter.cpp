/*
 * File: prpxy_starter.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 14th July 2019 7:01:14 pm
 */

#include "proxy/proxy_starter.h"

using namespace mellocolate;
// For readable seconds
using namespace std::chrono_literals;
using namespace std::string_literals;

// For websocket
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

ProxyStarter::ProxyStarter(std::string carla_ip, uint16_t carla_port,
                           std::string ws_ip, uint16_t ws_port)
    : carla_ip_(std::move(carla_ip)),
      carla_port_(carla_port),
      ws_ip_(std::move(ws_ip)),
      ws_port_(ws_port) {}

void ProxyStarter::Init() {
  try {
    LOG_INFO("Connecting to Carla Server on %s:%u...", carla_ip_.c_str(),
             carla_port_);
    carla_client_ptr_ =
        boost::make_shared<carla::client::Client>(carla_ip_, carla_port_);
    carla_client_ptr_->SetTimeout(10s);
    std::string server_version = carla_client_ptr_->GetServerVersion();
    if (carla_client_ptr_ == nullptr) {
      LOG_ERROR("Carla client ptr is null");
    } else {
      LOG_INFO("Connected to Carla Server, Server version is: %s",
               server_version.c_str());
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void ProxyStarter::Accpet() {
  LOG_INFO("Waiting for a client to connect. Listening to port %u....",
           ws_port_);
  try {
    boost::asio::io_context ioc{1};
    // TODO change ip here
    tcp::acceptor acceptor{ioc, tcp::endpoint(tcp::v4(), ws_port_)};
    for (;;) {
      tcp::socket socket{ioc};
      acceptor.accept(socket);
      auto t = std::thread(&ProxyStarter::AddClient, this, std::move(socket));
      t.detach();
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}
void ProxyStarter::AddClient(tcp::socket socket) {
  try {
    Proxy proxy(carla_client_ptr_, std::move(socket));
    proxy.Run();
  } catch (const std::exception& e) {
    LOG_WARNING(
        "Disconnected with frontend due to: [%s] .Please try again by "
        "refreshing browser or re-launching this proxy.",
        e.what());
  }
}

void ProxyStarter::Run() {
  Init();
  Accpet();
}

int main() {
  ProxyStarter proxy_starter;
  proxy_starter.Run();
  return 0;
}