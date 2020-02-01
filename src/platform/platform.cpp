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
  Init();

  while (true) {
    auto xviz = carla_proxy_->GetUpdateData();

    drawing_proxy_->AddDrawings(xviz);

    std::string output;
    xviz::XVIZGLBWriter writer;
    writer.WriteMessage(output, xviz.GetMessage());

    frontend_proxy_->SendToAllClients(output);

  }
}

void Platform::Clear() {
  LOG_INFO("Start to clean all resources. Don't forcefully exit!");
  if (carla_proxy_ != nullptr) {
    carla_proxy_->Clear();
  }
  LOG_INFO("All clear, exit!");
}

void Platform::Init() {
  drawing_proxy_ = std::make_shared<DrawingProxy>(8089u);
  drawing_proxy_->StartListen();

  frontend_proxy_ = std::make_shared<FrontendProxy>(8081u);
  frontend_proxy_->StartListen();

  carla_proxy_ = std::make_shared<CarlaProxy>("localhost", 2000u);
  carla_proxy_->Init();

  frontend_proxy_->UpdateMetadata(carla_proxy_->GetMetaData());
}

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  platform.Run();
}
