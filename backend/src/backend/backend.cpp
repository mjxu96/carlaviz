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

void Backend::SetIsExperimental(bool is_experimental) {
  is_experimental_ = is_experimental;
}

void Backend::SetTimeInterval(uint32_t time_interval) {
  time_interval_ = time_interval;
}

void Backend::Run() {
  try {
    if (is_experimental_) {
      LOG_INFO("Using experimental server");
      server_->Serve();
    } else {
      while (true) {
        auto xviz = carla_proxy_->GetUpdateData();

        drawing_proxy_->AddDrawings(xviz);

        std::string output;
        xviz::XVIZGLBWriter writer;
        writer.WriteMessage(output, xviz.GetMessage());

        frontend_proxy_->SendToAllClients(std::move(output));
      }
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
    backend.Clear();
  }

}

void Backend::Clear() {
  LOG_INFO("Start to clean all resources. Don't forcefully exit unless it takes too long!");
  if (carla_proxy_ != nullptr) {
    carla_proxy_->Clear();
  }
  LOG_INFO("All clear, exit! You may now forcefully exit if this program does not exit normally.");
}

void Backend::Init() {

  carla_proxy_ = std::make_shared<CarlaProxy>("localhost", 2000u, is_experimental_);
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
  if (argc > 1) {
    backend.SetIsExperimental(true);
    uint32_t ti = std::stoi(std::string(argv[1]));
    if (ti < 50u) {
      LOG_WARNING("The value %u is too small, reset to 50", ti);
      ti = 50u;
    }
    backend.SetTimeInterval(ti);
  }
  backend.Init();
  backend.Run();
}
