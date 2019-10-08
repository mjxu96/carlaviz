/*
 * File: platform.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:20:10 pm
 */
#include "platform/platform.h"

using namespace mellocolate;

void Platform::Run() {
  Init();

  while (true) {
    auto xviz = carla_proxy_->GetUpdateData();
    auto polylines = drawing_proxy_->GetPolyLines();
    XVIZPrimitiveBuider polyline_builder("/planning/trajectory");
    for (const auto& polyline : polylines) {
      polyline_builder.AddPolyLine(
          XVIZPrimitivePolyLineBuilder(polyline.second));
    }
    xviz.AddPrimitive(polyline_builder);
    frontend_proxy_->SendToAllClients(xviz.GetData());
  }
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
  Platform platform;
  platform.Run();
}
