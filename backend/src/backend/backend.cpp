/*
 * File: backend.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:20:10 pm
 */
#include "backend/backend.h"

#include <csignal>

carlaviz::Backend backend;

void signal_handler(int signal_num) {
  backend.Clear();
  std::abort();
}

using namespace carlaviz;

double GetTime(std::chrono::high_resolution_clock::time_point& t1, 
  std::chrono::high_resolution_clock::time_point& t2) {
  return std::chrono::duration<double, std::milli>(t2 - t1).count();
}

void Backend::SetIsExperimental(bool is_experimental) {
  is_experimental_ = is_experimental;
}

void Backend::SetTimeInterval(uint32_t time_interval) {
  time_interval_ = time_interval;
}

void Backend::SetCarlaHostAndPort(const std::string& host, uint16_t port) {
  carla_host_ = host;
  carla_port_ = port;
}

void Backend::Run() {
  try {
    if (is_experimental_) {
      CARLAVIZ_LOG_INFO("Using experimental server");
      server_->Serve();
    } else {
      while (true) {
        // auto start = std::chrono::high_resolution_clock::now();
        auto xviz = carla_proxy_->GetUpdateData();
        // auto data_end = std::chrono::high_resolution_clock::now();

        drawing_proxy_->AddDrawings(xviz);
        // auto drawing_end = std::chrono::high_resolution_clock::now();

        std::string output;
        xviz::XVIZGLBWriter writer;
        writer.WriteMessage(output, xviz.GetMessage());
        // auto write_end = std::chrono::high_resolution_clock::now();

        frontend_proxy_->SendToAllClients(std::move(output));
        // auto send_end = std::chrono::high_resolution_clock::now();
        /*
        CARLAVIZ_LOG_INFO("DATA TIME: %.3f", GetTime(start, data_end));
        CARLAVIZ_LOG_INFO("DRAWING TIME: %.3f", GetTime(data_end, drawing_end));
        CARLAVIZ_LOG_INFO("WRITER TIME: %.3f", GetTime(drawing_end, write_end));
        CARLAVIZ_LOG_INFO("SEND TIME: %.3f", GetTime(write_end, send_end));
        CARLAVIZ_LOG_INFO("TOTAL TIME: %.3f", GetTime(start, send_end));
        CARLAVIZ_LOG_INFO("-----------------------");
        */
      }
    }
  } catch (const std::exception& e) {
    CARLAVIZ_LOG_ERROR("%s", e.what());
    backend.Clear();
  }

}

void Backend::Clear() {
  CARLAVIZ_LOG_INFO("Start to clean all resources. Don't forcefully exit unless it takes too long!");
  if (carla_proxy_ != nullptr) {
    carla_proxy_->Clear();
  }
  CARLAVIZ_LOG_INFO("All clear, exit! You may now forcefully exit if this program does not exit normally.");
}

void Backend::Init() {

  carla_proxy_ = std::make_shared<CarlaProxy>(carla_host_, carla_port_, is_experimental_);
  carla_proxy_->Init();

  drawing_proxy_ = std::make_shared<DrawingProxy>(8089u);
  drawing_proxy_->StartListen();

  if (is_experimental_) {
    std::vector<std::shared_ptr<xviz::XVIZBaseHandler>> handlers;
    auto carla_handler = std::make_shared<carlaviz::CarlaHandler>(carla_proxy_, drawing_proxy_, time_interval_); 
    carla_handler->SetStreamSettingsCallback(std::bind(
      &CarlaProxy::SetTransmissionStreams, carla_proxy_, std::placeholders::_1
    ));
    handlers.push_back(carla_handler);
    server_ = std::make_shared<xviz::XVIZServer>(std::move(handlers));
    carla_proxy_->SetUpdateMetadataCallback(std::bind(
      &CarlaHandler::UpdateMetadata, carla_handler, std::placeholders::_1
    ));
  } else {
    frontend_proxy_ = std::make_shared<FrontendProxy>(8081u);
    frontend_proxy_->StartListen();
    frontend_proxy_->SetMapString(carla_proxy_->GetMapString());
    frontend_proxy_->UpdateMetadata(carla_proxy_->GetMetadata());
    frontend_proxy_->SetStreamSettingsCallback(std::bind(
      &CarlaProxy::SetTransmissionStreams, carla_proxy_, std::placeholders::_1
    ));
    carla_proxy_->SetUpdateMetadataCallback(std::bind(
      &FrontendProxy::UpdateMetadata, frontend_proxy_, std::placeholders::_1
    ));
  }
}

int main(int argc, char** argv) {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  if (argc > 2) {
    // CARLAVIZ_LOG_INFO("You are running Carla simulator on %s:%s", argv[1], argv[2]);
    uint16_t port = (uint16_t) std::stoi(argv[2]);
    backend.SetCarlaHostAndPort(argv[1], port);
  }
  backend.Init();
  backend.Run();
}
