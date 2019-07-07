/*
 * File: proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 10:11:52 pm
 */

#include "connector/proxy.h"

using namespace mellocolate;
// For readable seconds
using namespace std::chrono_literals;
using namespace std::string_literals;

// For websocket
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

Proxy::Proxy(std::string carla_host, uint16_t carla_port, uint16_t ws_port)
    : carla_host_(std::move(carla_host)),
      carla_port_(carla_port),
      ws_port_(ws_port) {}

void Proxy::Run() { 
  Init(); 
  while (true) {
    Update();
    std::this_thread::sleep_for(100ms);
  }
}

void Proxy::Init() {
  try {
    // Connect to Carla server
    carla::client::Client client(carla_host_, carla_port_);
    client.SetTimeout(10s);
    LOG_INFO("Connecting to Carla Server on %s:%u...", carla_host_.c_str(),
             carla_port_);
    //world_ptr_ = boost::make_shared<carla::client::World>(client.GetWorld());
    LOG_INFO("Connected to Carla Server");

    ws_accept_thread_ = std::thread(&Proxy::Accept, this);
    ws_accept_thread_.detach();

  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void Proxy::Update() {

  boost::beast::multi_buffer buffer;
  
  std::string update_data = "Hello";
  boost::beast::ostream(buffer) << update_data;

  // Update to all clients
  std::vector<std::shared_ptr<websocket::stream<tcp::socket>>> to_delete_ws;
  std::lock_guard<std::mutex> lock_gurad(ws_lock_);
  for (const auto& ws : ws_set_) {
    try {
      ws->write(buffer.data());
    } catch (const std::exception& e) {
      LOG_ERROR("%s", e.what());
      to_delete_ws.push_back(ws);
    }
  }
  //ws.write(buffer.data());
}

void Proxy::Accept() {
  LOG_INFO("Connecting to frontend client. Listening to port %u....", ws_port_);
  try {
    boost::asio::io_context ioc{1};

    tcp::acceptor acceptor{ioc, tcp::endpoint(tcp::v4(), ws_port_)};
    for (;;) {
      tcp::socket socket{ioc};

      acceptor.accept(socket);

      AddClient(std::move(socket));
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void Proxy::AddClient(tcp::socket socket) {
  std::shared_ptr<websocket::stream<tcp::socket>> ws_ptr =
      std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
  try {
    std::lock_guard<std::mutex> lock_gurad(ws_lock_);
    ws_ptr->accept();
    LOG_INFO("Client connected.");
    ws_set_.insert(ws_ptr);
  } catch (std::exception const& e) {
    std::lock_guard<std::mutex> lock_gurad(ws_lock_);
    if (ws_set_.find(ws_ptr) != ws_set_.end()) {
      ws_set_.erase(ws_ptr);
    }
    LOG_ERROR("%s", e.what());
  }
}

int main() {
  Proxy proxy;
  proxy.Run();
  return 0;
}