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

Proxy::Proxy(std::string carla_host, uint16_t carla_port, uint16_t ws_port) :
  carla_host_(std::move(carla_host)), carla_port_(carla_port), ws_port_(ws_port) {}


void Proxy::Run() {
  Init();
}

void Proxy::Init() {
  try {
    // Connect to Carla server
    carla::client::Client client(carla_host_, carla_port_);
    client.SetTimeout(10s);
    LOG_INFO("Connecting to Carla Server on %s:%u", carla_host_.c_str(), carla_port_);
    world_ptr_ = boost::make_shared<carla::client::World>(client.GetWorld());
    LOG_INFO("Connected to Carla Server");


  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void Proxy::Update() {

}

int main() {
  Proxy proxy;
  proxy.Run();
  return 0;
}