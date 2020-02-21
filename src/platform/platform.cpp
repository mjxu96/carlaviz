/*
 * File: platform.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:20:10 pm
 */
#include "platform/platform.h"

#include <csignal>

mellocolate::Platform platform;

void signal_handler(int signal_num) {
  platform.Clear();
  exit(0);
}

using namespace mellocolate;

void Platform::Run() {
  server_->Serve();
}

void Platform::Clear() {
  LOG_INFO("Start to clean all resources. Don't forcefully exit!");
  if (carla_proxy_ != nullptr) {
    carla_proxy_->Clear();
  }
  LOG_INFO("All clear, exit! You may now forcefully exit if this program does not exit normally.");
}

void Platform::Init() {
  drawing_proxy_ = std::make_shared<DrawingProxy>(8089u);
  drawing_proxy_->StartListen();

  // frontend_proxy_ = std::make_shared<FrontendProxy>(8081u);
  // frontend_proxy_->StartListen();

  carla_proxy_ = std::make_shared<CarlaProxy>("localhost", 2000u);
  carla_proxy_->Init();
  carla_proxy_->UpdateMetaData();
  std::thread t(std::bind(&CarlaProxy::UpdateData, carla_proxy_));
  t.detach();

  std::vector<std::shared_ptr<xviz::XVIZBaseHandler>> handlers;
  handlers.push_back(std::make_shared<mellocolate::CarlaHandler>(carla_proxy_, drawing_proxy_, 100u));
  server_ = std::make_shared<xviz::XVIZServer>(std::move(handlers));
}

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  platform.Init();
  platform.Run();
}
