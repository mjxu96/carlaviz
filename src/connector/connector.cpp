
#include "connector/utils/package.h"
#include "connector/websocket_server.h"

#include "connector/utils/xodr_geojson_converter.h"

#include "carla/client/Client.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/World.h"

#include "boost/shared_ptr.hpp"

#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::string_literals;

void RunWebsocketServer(rothberg::WebsocketServer& socket_server) {
  socket_server.Run();
}

int main() {
  try {
    std::string host("localhost");
    uint16_t port(2000u);

    std::cout << "[Connector Log] Connecting Carla Server..." << std::endl; 
    carla::client::Client client(host, port);
    client.SetTimeout(10s);

    //rothberg::utils::Package package(boost::make_shared<carla::client::World>(client.GetWorld()));
    boost::shared_ptr<rothberg::utils::Package> package_ptr = 
      boost::make_shared<rothberg::utils::Package>(boost::make_shared<carla::client::World>(client.GetWorld()));
    boost::shared_ptr<std::mutex> mutex_ptr = boost::make_shared<std::mutex>();

    std::cout << "[Connector Log] Connected with Carla Server!" << std::endl; 

    std::string geojson_str = rothberg::utils::XodrGeojsonConverter::GetGeoJsonFromCarlaMap(client.GetWorld().GetMap());

/*
  std::ofstream geojson_file("example.geojson", std::ios::out | std::ios::trunc);
  if (geojson_file.is_open()) {
    geojson_file << geojson_str;
    geojson_file.close();
  } else {
    std::cerr << "Not open" << std::endl;
  }
  */
  //XodrGeojsonConverter::Convert(buffer.str());

    std::cout << "[Connector Log] Starting Websocket Server for data transmission..." << std::endl; 
    rothberg::WebsocketServer socket_server(geojson_str);
    socket_server.Init(package_ptr, mutex_ptr);
    std::thread t = std::thread{std::bind(&RunWebsocketServer, 
      std::move(socket_server))};
    t.detach();

    while (true) {
      //auto time1 = std::chrono::system_clock::now();
      mutex_ptr->lock();
      package_ptr->Update();
      mutex_ptr->unlock();
      //packageTmpOutput();
      //auto time2 = std::chrono::system_clock::now();
      //std::chrono::duration<double> durat = time2 - time1;
      //std::cout << "use: " << durat.count() << std::endl;
      std::this_thread::sleep_for(40ms);
    }

  } catch (const carla::client::TimeoutException& e) {
    std::cout << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cout << "Exception: " << e.what() << std::endl;
    return 2;
  }
  return 0;
}